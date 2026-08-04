# Writes flash_images.txt next to firmware.bin listing every image the upload
# would write, as "<offset> <path>" lines. The release workflow feeds these to
# esptool merge-bin, so the merged image always matches what this build links
# against instead of a bootloader guessed from the packages directory.

Import("env")
import os


def dump_flash_images(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    rows = []

    # forward slashes so the paths stay usable from shell scripts on Windows too
    def norm(p):
        return os.path.abspath(p).replace("\\", "/")

    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        rows.append((env.subst(offset), norm(env.subst(image))))

    app_offset = env.subst("$ESP32_APP_OFFSET") or "0x10000"
    rows.append((app_offset, norm(os.path.join(build_dir, "firmware.bin"))))

    rows.sort(key=lambda r: int(r[0], 16))

    # newline="\n" so the file is LF even on Windows, it is parsed by shell
    with open(os.path.join(build_dir, "flash_images.txt"), "w", newline="\n") as f:
        for offset, path in rows:
            f.write("%s %s\n" % (offset, path))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", dump_flash_images)
