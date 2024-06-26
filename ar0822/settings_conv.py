import click
import re


def n_shifts(number: str) -> int:
    mask = int(number, 16)
    cntr = 0
    while not (mask % 2):
        mask /= 2
        cntr += 1
    return cntr


@click.command()
@click.option("--settings", required=True, help="Specify input ini file")
@click.option("--output", default="sensor_config.h", help="Output C header file.")
def run(settings, output):
    with open(settings, "r") as f:
        lines = f.readlines()

    settings = []

    for line in lines:
        matches = re.findall(r"0x[0-9A-Fa-f]+", line)
        if matches:
            if "REG = " in line:
                matches.append("0xFFFF")
            if "BITFIELD = " in line:
                matches[2] = f"0x{int(matches[2], 16) << n_shifts(matches[1]):04X}"
            settings.append(f"{{{', '.join(matches[:3])}}},\n")
            print(settings[-1], end="")

    with open(output, "w+") as f:
        f.write("#ifndef __SEN_CONF_H__\n#define __SEN_CONF_H__\n\n")
        f.write('#include "ar0822.h"\n\n')
        f.write(f"#define AR_CONF_SETTINGS_LEN ({len(settings)})\n\n")
        f.write("const ar_reg_write_t sensor_conf[AR_CONF_SETTINGS_LEN] = {\n")
        for setting in settings:
            f.write(f"\t{setting}")
        f.write("};\n\n")
        f.write("#endif\n\n")

    # with open(output, "w+") as f:
    #     f.write(f"#define FPGA_BITSTREAM_LEN ({bit_len})\n")
    #     f.write(f"const uint8_t bitstream[{bit_len}] = ")
    #     f.write("{")
    #     for line in lines[:-1]:
    #         f.write("0x" + line.strip("\n") + ", ")
    #     f.write("0x" + lines[-1].strip("\n") + "};\n")


if __name__ == "__main__":
    run()
