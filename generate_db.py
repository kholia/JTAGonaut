#!/usr/bin/env python3
import re
import os
import sys

def generate_db(input_path, output_path):
    if not os.path.exists(input_path):
        print(f"Error: Could not find {input_path}")
        return

    with open(input_path, "r") as f:
        content = f.read()

    fpga_entries = []
    mfg_entries = []

    # 1. Extract fpga_list
    # Pattern: {0x04014c35, {"anlogic", "eagle d20", "EG4D20EG176", 8}},
    # Using triple quotes to safely handle double quotes in the regex
    fpga_regex = r'''\{(0x[0-9a-fA-F]+),\s*\{"([^"]+)",\s*"([^"]+)",\s*"([^"]+)",\s*(\d+)\}\}'''
    fpga_matches = re.finditer(fpga_regex, content)
    for m in fpga_matches:
        fpga_entries.append({
            "idcode": int(m.group(1), 16),
            "mfg": m.group(2),
            "family": m.group(3),
            "model": m.group(4),
            "irlen": int(m.group(5))
        })

    # 2. Extract manufacturer list
    # Pattern: {0x61a, "anlogic"},
    mfg_regex = r'''\{(0x[0-9a-fA-F]+),\s*"([^"]+)"\}'''
    mfg_matches = re.finditer(mfg_regex, content)
    for m in mfg_matches:
        mfg_entries.append({
            "id": int(m.group(1), 16),
            "name": m.group(2)
        })

    # Sort for consistent output and easier lookups
    fpga_entries.sort(key=lambda x: x["idcode"])
    mfg_entries.sort(key=lambda x: x["id"])

    with open(output_path, "w") as f:
        f.write("// JTAGonaut FPGA Database\n")
        f.write("// Automatically generated from openFPGALoader (part.hpp)\n")
        f.write("#ifndef FPGA_DB_H\n")
        f.write("#define FPGA_DB_H\n\n")

        f.write("struct fpga_entry {\n")
        f.write("  uint32_t idcode;\n")
        f.write("  const char* mfg;\n")
        f.write("  const char* family;\n")
        f.write("  const char* model;\n")
        f.write("  uint8_t irlen;\n")
        f.write("};\n\n")

        f.write("struct mfg_entry {\n")
        f.write("  uint16_t id;\n")
        f.write("  const char* name;\n")
        f.write("};\n\n")

        # FPGA Table
        f.write(f"const int FPGA_COUNT = {len(fpga_entries)};\n")
        f.write("const fpga_entry fpga_table[] PROGMEM = {\n")
        for e in fpga_entries:
            f.write(f"  {{ 0x{e['idcode']:08x}, \"{e['mfg']}\", \"{e['family']}\", \"{e['model']}\", {e['irlen']} }},\n")
        f.write("};\n\n")

        # Manufacturer Table
        f.write(f"const int MFG_COUNT = {len(mfg_entries)};\n")
        f.write("const mfg_entry mfg_table[] PROGMEM = {\n")
        for e in mfg_entries:
            f.write(f"  {{ 0x{e['id']:03x}, \"{e['name']}\" }},\n")
        f.write("};\n\n")

        f.write("#endif // FPGA_DB_H\n")

    print(f"Successfully generated {output_path}")
    print(f"Total FPGAs: {len(fpga_entries)}")
    print(f"Total Manufacturers: {len(mfg_entries)}")

if __name__ == "__main__":
    input_file = "openFPGALoader/src/part.hpp"
    output_file = "fpga_db.h"
    generate_db(input_file, output_file)
