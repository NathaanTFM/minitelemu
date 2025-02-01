#!/bin/bash
mkdir temp -p

BIN2C="import sys
data = open(sys.argv[1], 'rb').read()
open(sys.argv[2], 'w').write('extern const char ' + sys.argv[4] + '[];\nextern const int ' + sys.argv[4] + '_len;')
open(sys.argv[3], 'w').write('const char ' + sys.argv[4] + '[] = {' + ','.join(hex(i) for i in data) + '};\nconst int ' + sys.argv[4] + '_len = ' + str(len(data)) + ';')
"

python3 -c "$BIN2C" data/charset.bin temp/charset_bin.h temp/charset_bin.c charset_bin
python3 -c "$BIN2C" data/rom.bin temp/rom_bin.h temp/rom_bin.c rom_bin

mkdir web -p
emcc src/main_web.c src/minitel.c src/graphics.c src/modem.c src/mcu.c src/mcu_instr.c temp/charset_bin.c temp/rom_bin.c -Itemp -s USE_SDL=2 -o web/minitel.html -O3
ret=$?

rm -f temp/charset_bin.h temp/charset_bin.c temp/rom_bin.h temp/rom_bin.c temp/page_vdt.h temp/page_vdt.c
rmdir temp

exit $ret
