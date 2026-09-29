#!/usr/bin/env python3
import struct
import sys

M = 0xFFFFFFFF
SEED = 0x9E37A9EA
LCG = 0x41C64E6D
WEYL = 0x9E3779B1

DISK_OFF = 0x100000   # смещение диска внутри дампа флеша
SECTORS = 2048
SECSZ = 512


def sector_keystream(lba: int) -> bytes:
    """512 байт keystream для сектора lba."""
    ks = bytearray(SECSZ)
    for g in range(SECSZ // 16):
        state = (SEED + (lba * 32 + g) * LCG) & M
        for j in range(16):
            ks[g * 16 + j] = ((state + ((j - 1) & M) * WEYL) & M) >> 24
    return bytes(ks)


def decrypt(flash: bytes) -> bytes:
    out = bytearray(SECTORS * SECSZ)
    for lba in range(SECTORS):
        base = DISK_OFF + lba * SECSZ
        ks = sector_keystream(lba)
        for i in range(SECSZ):
            out[lba * SECSZ + i] = flash[base + i] ^ ks[i]
    return bytes(out)


def encrypt(plain_disk: bytes) -> bytes:
    """Обратная операция (для патча диска): plaintext -> ciphertext.
    XOR симметричен, так что это тот же keystream."""
    out = bytearray(len(plain_disk))
    for lba in range(SECTORS):
        ks = sector_keystream(lba)
        for i in range(SECSZ):
            out[lba * SECSZ + i] = plain_disk[lba * SECSZ + i] ^ ks[i]
    return bytes(out)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "full.bin"
    dst = sys.argv[2] if len(sys.argv) > 2 else "disk.img"
    flash = open(src, "rb").read()
    if len(flash) < DISK_OFF + SECTORS * SECSZ:
        sys.exit(f"дамп слишком мал: {len(flash)} байт")

    img = decrypt(flash)
    open(dst, "wb").write(img)

    ok = img[:3] == b"\xeb\x3c\x90"
    print(f"[+] {dst} записан ({len(img)} байт)")
    print(f"[+] FAT boot sector: {'OK' if ok else 'НЕ РАСПОЗНАН'} "
          f"(OEM={img[3:11].decode('latin1', 'replace')!r}, "
          f"label={img[0x2b:0x36].decode('latin1', 'replace')!r})")
    if not ok:
        sys.exit("расшифровка не дала корректный FAT — проверь смещение/константы")


if __name__ == "__main__":
    main()
