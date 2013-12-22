/*
 * StelVideo: video signal generator for the Stellaris LaunchPad.
 * img2fb is a tool for converting images for display, for demo purposes.
 * Copyright (C) 2013 Boris Gjenero <boris.gjenero@gmail.com>
 *
 * StelVideo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * StelVideo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with StelVideo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdio.h>

unsigned char bitreverse(unsigned char c)
{
    int i;
    unsigned char r = 0;
    for (i = 0; i < 8; i++) {
        r <<= 1;
        if (c & 1) r |= 1;
        c >>= 1;
    }
    return r;
}

void buildrevtab(unsigned char revtab[256])
{
    int i;
    for (i = 0; i < 256; i++) {
        revtab[i] = bitreverse(i);
    }
}

int main(int argc, char **argv)
{
    FILE *f;
    unsigned char pair[2];
    int got;
    unsigned char revtab[256];
    int col = 0;

    if (argc != 2) {
        fprintf(stderr,
                "USAGE: %s IMAGE.mono\n"
                "IMAGE.mono may be created via ImageMagick convert.\n"
                "Image must be of the appropriate size.", argv[0]);
        exit(1);
    }

    f = fopen(argv[1], "rb");
    if (f == NULL) {
        perror("Couldn't open file");
        exit(1);
    }

    printf("unsigned char fb[H_PIXELS*V_DISPLINES/8] = {\n");

    buildrevtab(revtab);

    while (1) {
        got = fread(pair, 1, 2, f);
        if (got != 2) break;
        printf("0x%02x,0x%02x,", revtab[pair[1]], revtab[pair[0]]);
        if (col == 5) {
            col = 0;
            printf("\n");
        } else {
            col++;
        }
    }

    if (col != 0)
        printf("\n");
    printf("};\n");
    return 0;
}
