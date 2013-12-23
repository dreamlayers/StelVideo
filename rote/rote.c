/*
LICENSE INFORMATION:
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License (LGPL) as published by the Free Software Foundation.

Please refer to the COPYING file for more information.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

Copyright (c) 2004 Bruno T. C. de Oliveira
*/


#include "rote.h"
#include "roteprivate.h"
#include <string.h>
#if 0
#include <pty.h>
#include <stdio.h>
#endif

#define ROTE_VT_UPDATE_ITERATIONS 5

static RoteTerm rote_term;
static RoteTermPrivate rote_private;

RoteTerm *rote_vt_create(int rows, int cols,
                         RoteCell **rowptrs, RoteCell *cells, bool *dirty) {
   RoteTerm *rt;
   int i, j;

   if (rows <= 0 || cols <= 0) return NULL;

   rt = &rote_term;
   memset(rt, 0, sizeof(RoteTerm));

   /* record dimensions */
   rt->rows = rows;
   rt->cols = cols;

   /* create the cell matrix */
   rt->cells = rowptrs;
   for (i = 0; i < rt->rows; i++) {
      /* create row */
      rt->cells[i] = &cells[i * cols];

      /* fill row with spaces */
      for (j = 0; j < rt->cols; j++) {
         rt->cells[i][j].ch = 0x20;    /* a space */
         rt->cells[i][j].attr = 0x70;  /* white text, black background */
      }
   }
   
   /* allocate dirtiness array */
   rt->line_dirty = dirty;

   /* initialization of other public fields */
   rt->crow = rt->ccol = 0;
   rt->curattr = 0x70;  /* white text over black background */

   /* allocate private data */
   rt->pd = &rote_private;
   memset(rt->pd, 0, sizeof(RoteTermPrivate));

   rt->pd->pty = -1;  /* no pty for now */

   /* initial scrolling area is the whole window */
   rt->pd->scrolltop = 0;
   rt->pd->scrollbottom = rt->rows - 1;

   #ifdef DEBUG
   fprintf(stderr, "Created a %d x %d terminal.\n", rt->rows, rt->cols);
   #endif
   
   return rt;
}

void rote_vt_destroy(RoteTerm *rt) {
   if (!rt) return;
}

void rote_vt_install_handler(RoteTerm *rt, rote_es_handler_t handler) {
   rt->pd->handler = handler;
}
