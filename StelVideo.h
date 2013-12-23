/*
 * StelVideo: video signal generator for the Stellaris LaunchPad.
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

/* #define PART_LM4F120H5QR */

extern unsigned char text_screen[];

void uart_init(void);
void uart_run(void);
