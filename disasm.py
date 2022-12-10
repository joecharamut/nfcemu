Import("env")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", env.VerboseAction(
    " ".join([
        "avr-objdump", "--disassemble", "--demangle", "--line-numbers", "-M", "intel", 
        "$BUILD_DIR/${PROGNAME}.elf", ">$BUILD_DIR/${PROGNAME}.elf.lst"
    ]),
    "Generating $BUILD_DIR/${PROGNAME}.elf.lst"
))
