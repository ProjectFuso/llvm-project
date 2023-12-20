//===--- Fuso.cpp - Project Fuso ToolChain Implementations ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Fuso.h"
#include "CommonArgs.h"
#include "clang/Basic/LangStandard.h"
#include "clang/Config/config.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Driver/Options.h"
#include "clang/Driver/SanitizerArgs.h"
#include "clang/Driver/ToolChain.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"

using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;
using namespace clang;
using namespace llvm::opt;

void fuso::Assembler::ConstructJob(Compilation &C, const JobAction &JA,
                                   const InputInfo &Output,
                                   const InputInfoList &Inputs,
                                   const ArgList &Args,
                                   const char *LinkingOutput) const {
  claimNoWarnArgs(Args);
  ArgStringList CmdArgs;

  Args.AddAllArgValues(CmdArgs, options::OPT_Wa_COMMA, options::OPT_Xassembler);

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  for (const auto &II : Inputs)
    CmdArgs.push_back(II.getFilename());

  const char *Exec = Args.MakeArgString(getToolChain().GetProgramPath("as"));
  C.addCommand(std::make_unique<Command>(JA, *this, ResponseFileSupport::None(),
                                         Exec, CmdArgs, Inputs, Output));
}

void fuso::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                                const InputInfo &Output,
                                const InputInfoList &Inputs,
                                const ArgList &Args,
                                const char *LinkingOutput) const {
  const toolchains::Generic_ELF &ToolChain =
      static_cast<const toolchains::Generic_ELF &>(getToolChain());
  ToolChain::RuntimeLibType RuntimeLib = ToolChain.GetRuntimeLibType(Args);

  ArgStringList CmdArgs;

  if (!Args.hasArg(options::OPT_nostdlib, options::OPT_shared)) {
    CmdArgs.push_back("-e");
    CmdArgs.push_back("_start");
  }

  if (Args.hasArg(options::OPT_static)) {
    CmdArgs.push_back("-Bstatic");
    CmdArgs.push_back("-dn");
  } else {
    CmdArgs.push_back("-Bdynamic");
    if (Args.hasArg(options::OPT_shared)) {
      CmdArgs.push_back("-shared");
    }

    if (!Args.hasArg(options::OPT_shared) && !Args.hasArg(options::OPT_r)) {
      CmdArgs.push_back("-dynamic-linker");
      CmdArgs.push_back(Args.MakeArgString(ToolChain.getDynamicLinker(Args)));
    }

    // libpthread has been folded into libc since Solaris 10, no need to do
    // anything for pthreads. Claim argument to avoid warning.
    Args.ClaimAllArgs(options::OPT_pthread);
    Args.ClaimAllArgs(options::OPT_pthreads);
  }

  if (Output.isFilename()) {
    CmdArgs.push_back("-o");
    CmdArgs.push_back(Output.getFilename());
  } else {
    assert(Output.isNothing() && "Invalid output.");
  }

  if (!Args.hasArg(options::OPT_nostdlib, options::OPT_nostartfiles,
                   options::OPT_r)) {
    if (!Args.hasArg(options::OPT_shared))
      CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath("crt1.o")));

    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath("crti.o")));

    const Arg *Std = Args.getLastArg(options::OPT_std_EQ, options::OPT_ansi);
    bool HaveAnsi = false;
    const LangStandard *LangStd = nullptr;
    if (Std) {
      HaveAnsi = Std->getOption().matches(options::OPT_ansi);
      if (!HaveAnsi)
        LangStd = LangStandard::getLangStandardForName(Std->getValue());
    }

    const char *values_X = "values-Xa.o";
    // Use values-Xc.o for -ansi, -std=c*, -std=iso9899:199409.
    if (HaveAnsi || (LangStd && !LangStd->isGNUMode()))
      values_X = "values-Xc.o";
    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath(values_X)));

    const char *values_xpg = "values-xpg6.o";
    // Use values-xpg4.o for -std=c90, -std=gnu90, -std=iso9899:199409.
    if (LangStd && LangStd->getLanguage() == Language::C && !LangStd->isC99())
      values_xpg = "values-xpg4.o";
    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath(values_xpg)));

    std::string crtbegin;
    if (RuntimeLib == ToolChain::RLT_CompilerRT) {
      crtbegin =
          ToolChain.getCompilerRT(Args, "crtbegin", ToolChain::FT_Object);
    }
    if (crtbegin.empty())
      crtbegin = ToolChain.GetFilePath("crtbegin.o");
    CmdArgs.push_back(Args.MakeArgString(crtbegin));

    // Add crtfastmath.o if available and fast math is enabled.
    ToolChain.addFastMathRuntimeIfAvailable(Args, CmdArgs);
  }

  ToolChain.AddFilePathLibArgs(Args, CmdArgs);

  Args.AddAllArgs(CmdArgs, {options::OPT_L, options::OPT_T_Group,
                            options::OPT_e, options::OPT_r});

  bool NeedsSanitizerDeps = addSanitizerRuntimes(getToolChain(), Args, CmdArgs);
  AddLinkerInputs(ToolChain, Inputs, Args, CmdArgs, JA);

  if (!Args.hasArg(options::OPT_nostdlib, options::OPT_nodefaultlibs,
                   options::OPT_r)) {
    if (ToolChain.ShouldLinkCXXStdlib(Args))
      ToolChain.AddCXXStdlibLibArgs(Args, CmdArgs);
    if (Args.hasArg(options::OPT_fstack_protector) ||
        Args.hasArg(options::OPT_fstack_protector_strong) ||
        Args.hasArg(options::OPT_fstack_protector_all)) {
      // Explicitly link ssp libraries, not folded into Solaris libc.
      CmdArgs.push_back("-lssp_nonshared");
      CmdArgs.push_back("-lssp");
    }

    CmdArgs.push_back("--as-needed");

    if (RuntimeLib == ToolChain::RLT_Libgcc)
      CmdArgs.push_back("-lgcc_s");

    CmdArgs.push_back("-lc");

    if (!Args.hasArg(options::OPT_shared)) {
      if (RuntimeLib == ToolChain::RLT_Libgcc)
        CmdArgs.push_back("-lgcc");
      CmdArgs.push_back("-lm");
    }

    if (NeedsSanitizerDeps) {
      linkSanitizerRuntimeDeps(getToolChain(), CmdArgs);
    }
  }

  if (!Args.hasArg(options::OPT_nostdlib, options::OPT_nostartfiles,
                   options::OPT_r)) {
    std::string crtend;
    if (RuntimeLib == ToolChain::RLT_CompilerRT)
      crtend = ToolChain.getCompilerRT(Args, "crtend", ToolChain::FT_Object);
    if (crtend.empty())
      crtend = ToolChain.GetFilePath("crtend.o");
    CmdArgs.push_back(Args.MakeArgString(crtend));

    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath("crtn.o")));
  }

  ToolChain.addProfileRTLibs(Args, CmdArgs);

  const char *Exec = Args.MakeArgString(ToolChain.GetLinkerPath());
  C.addCommand(std::make_unique<Command>(JA, *this, ResponseFileSupport::None(),
                                         Exec, CmdArgs, Inputs, Output));
}

static StringRef getFusoLibSuffix(const llvm::Triple &Triple) {
  switch (Triple.getArch()) {
  case llvm::Triple::x86:
  case llvm::Triple::sparc:
    break;
  case llvm::Triple::x86_64:
    return "/amd64";
  case llvm::Triple::sparcv9:
    return "/sparcv9";
  default:
    llvm_unreachable("Unsupported architecture");
  }
  return "";
}

/// Fuso - Project Fuso tool chain which can call as(1) and ld(1) directly.

Fuso::Fuso(const Driver &D, const llvm::Triple &Triple, const ArgList &Args)
    : Generic_ELF(D, Triple, Args) {

  StringRef LibSuffix = getFusoLibSuffix(Triple);
  path_list &Paths = getFilePaths();
  if (GetRuntimeLibType(Args) == ToolChain::RLT_Libgcc) {
    GCCInstallation.init(Triple, Args);
    if (GCCInstallation.isValid()) {
      // On Solaris gcc uses both an architecture-specific path with triple in
      // it as well as a more generic lib path (+arch suffix).
      addPathIfExists(D,
                      GCCInstallation.getInstallPath() +
                          GCCInstallation.getMultilib().gccSuffix(),
                      Paths);
      addPathIfExists(D, GCCInstallation.getParentLibPath() + LibSuffix, Paths);
    }
  }

  addPathIfExists(D, D.SysRoot + "/usr/lib" + LibSuffix, Paths);
}

SanitizerMask Fuso::getSupportedSanitizers() const {
  const bool IsX86 = getTriple().getArch() == llvm::Triple::x86;
  const bool IsX86_64 = getTriple().getArch() == llvm::Triple::x86_64;
  SanitizerMask Res = ToolChain::getSupportedSanitizers();
  // FIXME: Omit X86_64 until 64-bit support is figured out.
  if (IsX86) {
    Res |= SanitizerKind::Address;
    Res |= SanitizerKind::PointerCompare;
    Res |= SanitizerKind::PointerSubtract;
  }
  if (IsX86 || IsX86_64)
    Res |= SanitizerKind::Function;
  Res |= SanitizerKind::Vptr;
  return Res;
}

Tool *Fuso::buildAssembler() const { return new tools::fuso::Assembler(*this); }

Tool *Fuso::buildLinker() const { return new tools::fuso::Linker(*this); }

void Fuso::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                     ArgStringList &CC1Args) const {
  const Driver &D = getDriver();

  if (DriverArgs.hasArg(clang::driver::options::OPT_nostdinc))
    return;

  if (!DriverArgs.hasArg(options::OPT_nostdlibinc))
    addSystemInclude(DriverArgs, CC1Args, D.SysRoot + "/usr/local/include");

  if (!DriverArgs.hasArg(options::OPT_nobuiltininc)) {
    SmallString<128> P(D.ResourceDir);
    llvm::sys::path::append(P, "include");
    addSystemInclude(DriverArgs, CC1Args, P);
  }

  if (DriverArgs.hasArg(options::OPT_nostdlibinc))
    return;

  // Check for configure-time C include directories.
  StringRef CIncludeDirs(C_INCLUDE_DIRS);
  if (CIncludeDirs != "") {
    SmallVector<StringRef, 5> dirs;
    CIncludeDirs.split(dirs, ":");
    for (StringRef dir : dirs) {
      StringRef Prefix =
          llvm::sys::path::is_absolute(dir) ? "" : StringRef(D.SysRoot);
      addExternCSystemInclude(DriverArgs, CC1Args, Prefix + dir);
    }
    return;
  }

  // Add include directories specific to the selected multilib set and multilib.
  if (GCCInstallation.isValid()) {
    const MultilibSet::IncludeDirsFunc &Callback =
        Multilibs.includeDirsCallback();
    if (Callback) {
      for (const auto &Path : Callback(GCCInstallation.getMultilib()))
        addExternCSystemIncludeIfExists(
            DriverArgs, CC1Args, GCCInstallation.getInstallPath() + Path);
    }
  }

  addExternCSystemInclude(DriverArgs, CC1Args, D.SysRoot + "/usr/include");
}

void Fuso::AddCXXStdlibLibArgs(const llvm::opt::ArgList &Args,
                               llvm::opt::ArgStringList &CmdArgs) const {
  switch (GetCXXStdlibType(Args)) {
  case ToolChain::CST_Libstdcxx:
      if (!GCCInstallation.isValid()) {
	  // Needs GCC to provide libstdc++ on Project Fuso
	  return;
      }
      CmdArgs.push_back("-lstdc++");
      return;
  case ToolChain::CST_Libcxx:
      CmdArgs.push_back("-lc++");
      CmdArgs.push_back("-lc++abi");
      return;
  }
  llvm_unreachable("Unsupported C++ Standard Library");
}

void Fuso::addLibCxxIncludePaths(const llvm::opt::ArgList &DriverArgs,
                                 llvm::opt::ArgStringList &CC1Args) const {
  addSystemInclude(DriverArgs, CC1Args,
                   concat(getDriver().SysRoot, "/usr/include/c++/v1"));
}

void Fuso::addLibStdCxxIncludePaths(const llvm::opt::ArgList &DriverArgs,
                                    llvm::opt::ArgStringList &CC1Args) const {
  // We need a detected GCC installation on Project Fuso (similar to Linux)
  // to provide libstdc++'s headers.
  if (!GCCInstallation.isValid())
    return;

  // By default, look for the C++ headers in an include directory adjacent to
  // the lib directory of the GCC installation.
  // On Solaris this usually looks like /usr/gcc/X.Y/include/c++/X.Y.Z
  StringRef LibDir = GCCInstallation.getParentLibPath();
  StringRef TripleStr = GCCInstallation.getTriple().str();
  const Multilib &Multilib = GCCInstallation.getMultilib();
  const GCCVersion &Version = GCCInstallation.getVersion();

  // The primary search for libstdc++ supports multiarch variants.
  addLibStdCXXIncludePaths(LibDir.str() + "/../include/c++/" + Version.Text,
                           TripleStr, Multilib.includeSuffix(), DriverArgs,
                           CC1Args);
}

std::string Fuso::getDynamicLinker(const llvm::opt::ArgList &Args) const {
  const llvm::Triple &Triple = getTriple();
  std::string LibSuffix = getFusoLibSuffix(Triple).str();

  return "/usr/lib" + LibSuffix + "/ld.so.1";
}
