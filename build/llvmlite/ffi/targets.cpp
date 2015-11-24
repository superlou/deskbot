#include "core.h"
#include "llvm-c/Target.h"
#include "llvm/Support/Host.h"
#include "llvm-c/TargetMachine.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Target/TargetLibraryInfo.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/IR/Type.h"

#include <cstdio>
#include <cstring>


namespace llvm {


inline LLVMTargetLibraryInfoRef wrap(TargetLibraryInfo *TLI) {
    return reinterpret_cast<LLVMTargetLibraryInfoRef>(TLI);
}

inline TargetLibraryInfo *unwrap(LLVMTargetLibraryInfoRef TLI) {
    return reinterpret_cast<TargetLibraryInfo*>(TLI);
}

inline Target *unwrap(LLVMTargetRef T) {
    return reinterpret_cast<Target*>(T);
}

inline LLVMTargetMachineRef wrap(TargetMachine *TM) {
    return reinterpret_cast<LLVMTargetMachineRef>(TM);
}

}

extern "C" {

API_EXPORT(void)
LLVMPY_GetDefaultTargetTriple(const char **Out) {
    // Should we use getProcessTriple() instead?
    *Out = LLVMPY_CreateString(llvm::sys::getDefaultTargetTriple().c_str());
}

API_EXPORT(void)
LLVMPY_GetHostCPUName(const char **Out) {
    *Out = LLVMPY_CreateString(llvm::sys::getHostCPUName().data());
}

API_EXPORT(int)
LLVMPY_GetTripleObjectFormat(const char *tripleStr)
{
    return llvm::Triple(tripleStr).getObjectFormat();
}

API_EXPORT(LLVMTargetDataRef)
LLVMPY_CreateTargetData(const char *StringRep)
{
    return LLVMCreateTargetData(StringRep);
}

API_EXPORT(void)
LLVMPY_AddTargetData(LLVMTargetDataRef TD,
                     LLVMPassManagerRef PM)
{
    LLVMAddTargetData(TD, PM);
}


//// Nothing is creating a TargetLibraryInfo
//    void
//    LLVMPY_AddTargetLibraryInfo(LLVMTargetLibraryInfoRef TLI,
//                                LLVMPassManagerRef PM)
//    {
//        LLVMAddTargetLibraryInfo(TLI, PM);
//    }

API_EXPORT(void)
LLVMPY_CopyStringRepOfTargetData(LLVMTargetDataRef TD, char** Out)
{
    *Out = LLVMCopyStringRepOfTargetData(TD);
}

API_EXPORT(void)
LLVMPY_DisposeTargetData(LLVMTargetDataRef TD)
{
    LLVMDisposeTargetData(TD);
}


API_EXPORT(long long)
LLVMPY_ABISizeOfType(LLVMTargetDataRef TD, LLVMTypeRef Ty)
{
    return (long long) LLVMABISizeOfType(TD, Ty);
}

API_EXPORT(long long)
LLVMPY_ABISizeOfElementType(LLVMTargetDataRef TD, LLVMTypeRef Ty)
{
    llvm::Type *tp = llvm::unwrap(Ty);
    if (!tp->isPointerTy())
        return -1;
    tp = tp->getSequentialElementType();
    return (long long) LLVMABISizeOfType(TD, llvm::wrap(tp));
}

API_EXPORT(long long)
LLVMPY_ABIAlignmentOfElementType(LLVMTargetDataRef TD, LLVMTypeRef Ty)
{
    llvm::Type *tp = llvm::unwrap(Ty);
    if (!tp->isPointerTy())
        return -1;
    tp = tp->getSequentialElementType();
    return (long long) LLVMABIAlignmentOfType(TD, llvm::wrap(tp));
}


API_EXPORT(LLVMTargetRef)
LLVMPY_GetTargetFromTriple(const char *Triple, const char **ErrOut)
{
    char *ErrorMessage;
    LLVMTargetRef T;
    if (LLVMGetTargetFromTriple(Triple, &T, &ErrorMessage)) {
        *ErrOut = LLVMPY_CreateString(ErrorMessage);
        LLVMDisposeMessage(ErrorMessage);
        return NULL;
    }
    return T;
}

API_EXPORT(const char *)
LLVMPY_GetTargetName(LLVMTargetRef T)
{
    return LLVMGetTargetName(T);
}

API_EXPORT(const char *)
LLVMPY_GetTargetDescription(LLVMTargetRef T)
{
    return LLVMGetTargetDescription(T);
}

API_EXPORT(LLVMTargetMachineRef)
LLVMPY_CreateTargetMachine(LLVMTargetRef T,
                           const char *Triple,
                           const char *CPU,
                           const char *Features,
                           int         OptLevel,
                           const char *RelocModel,
                           const char *CodeModel,
                           int         EmitJITDebug,
                           int         PrintMC)
{
    using namespace llvm;
    CodeGenOpt::Level cgol;
    switch(OptLevel) {
    case 0:
        cgol = CodeGenOpt::None;
        break;
    case 1:
        cgol = CodeGenOpt::Less;
        break;
    case 3:
        cgol = CodeGenOpt::Aggressive;
        break;
    case 2:
    default:
        cgol = CodeGenOpt::Default;
    }

    CodeModel::Model cm;
    std::string cms(CodeModel);
    if (cms == "jitdefault")
        cm = CodeModel::JITDefault;
    else if (cms == "small")
        cm = CodeModel::Small;
    else if (cms == "kernel")
        cm = CodeModel::Kernel;
    else if (cms == "medium")
        cm = CodeModel::Medium;
    else if (cms == "large")
        cm = CodeModel::Large;
    else
        cm = CodeModel::Default;

    Reloc::Model rm;
    std::string rms(RelocModel);
    if (rms == "static")
        rm = Reloc::Static;
    else if (rms == "pic")
        rm = Reloc::PIC_;
    else if (rms == "dynamicnopic")
        rm = Reloc::DynamicNoPIC;
    else
        rm = Reloc::Default;

    TargetOptions opt;
    opt.JITEmitDebugInfo = EmitJITDebug;
    opt.PrintMachineCode = PrintMC;

    return wrap(unwrap(T)->createTargetMachine(Triple, CPU, Features, opt,
                                               rm, cm, cgol));
}


API_EXPORT(void)
LLVMPY_DisposeTargetMachine(LLVMTargetMachineRef TM)
{
    return LLVMDisposeTargetMachine(TM);
}

API_EXPORT(void)
LLVMPY_GetTargetMachineTriple(LLVMTargetMachineRef TM, const char **Out)
{
    // result is already strdup()ed by LLVMGetTargetMachineTriple
    *Out = LLVMGetTargetMachineTriple(TM);
}


API_EXPORT(LLVMMemoryBufferRef)
LLVMPY_TargetMachineEmitToMemory (
    LLVMTargetMachineRef TM,
    LLVMModuleRef M,
    int use_object,
    const char ** ErrOut
    )
{
    LLVMCodeGenFileType filetype = LLVMAssemblyFile;
    if (use_object) filetype = LLVMObjectFile;

    char *ErrorMessage;
    LLVMMemoryBufferRef BufOut;
    int err = LLVMTargetMachineEmitToMemoryBuffer(TM, M, filetype,
                                                  &ErrorMessage,
                                                  &BufOut);
    if (err) {
        *ErrOut = LLVMPY_CreateString(ErrorMessage);
        LLVMDisposeMessage(ErrorMessage);
        return NULL;
    }

    return BufOut;
}

API_EXPORT(LLVMTargetDataRef)
LLVMPY_GetTargetMachineData(LLVMTargetMachineRef TM)
{
    return LLVMGetTargetMachineData(TM);
}

API_EXPORT(void)
LLVMPY_AddAnalysisPasses(
    LLVMTargetMachineRef TM,
    LLVMPassManagerRef PM
    )
{
    LLVMAddAnalysisPasses(TM, PM);
}


API_EXPORT(const void*)
LLVMPY_GetBufferStart(LLVMMemoryBufferRef MB)
{
    return LLVMGetBufferStart(MB);
}

API_EXPORT(size_t)
LLVMPY_GetBufferSize(LLVMMemoryBufferRef MB)
{
    return LLVMGetBufferSize(MB);
}

API_EXPORT(void)
LLVMPY_DisposeMemoryBuffer(LLVMMemoryBufferRef MB)
{
    return LLVMDisposeMemoryBuffer(MB);
}


API_EXPORT(LLVMTargetLibraryInfoRef)
LLVMPY_CreateTargetLibraryInfo(const char *Triple)
{
    return llvm::wrap(new llvm::TargetLibraryInfo(llvm::Triple(Triple)));
}

API_EXPORT(void)
LLVMPY_DisposeTargetLibraryInfo(LLVMTargetLibraryInfoRef TLI)
{
    delete llvm::unwrap(TLI);
}

API_EXPORT(void)
LLVMPY_AddTargetLibraryInfo(
    LLVMTargetLibraryInfoRef TLI,
    LLVMPassManagerRef PM
    )
{
    LLVMAddTargetLibraryInfo(TLI, PM);
}

API_EXPORT(void)
LLVMPY_DisableAllBuiltins(LLVMTargetLibraryInfoRef TLI)
{
    llvm::unwrap(TLI)->disableAllFunctions();
}

API_EXPORT(int)
LLVMPY_GetLibFunc(LLVMTargetLibraryInfoRef TLI, const char *Name, int *OutF)
{
    llvm::LibFunc::Func F;
    if (llvm::unwrap(TLI)->getLibFunc(Name, F)) {
        /* Ok */
        *OutF = F;
        return 1;
    } else {
        /* Failed */
        return 0;
    }
}

API_EXPORT(void)
LLVMPY_SetUnavailableLibFunc(LLVMTargetLibraryInfoRef TLI, int F)
{
    llvm::unwrap(TLI)->setUnavailable((llvm::LibFunc::Func)F);
}


} // end extern "C"
