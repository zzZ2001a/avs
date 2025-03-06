/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
  Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
  Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
  Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. may not be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/

#include "dec_def.h"

#if DT_PARTITION
void pred_recon_intra_luma_pb(COM_PIC *pic, u32 *map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, s16 resi[N_C][MAX_CU_DIM], pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE *mod_info_curr,
                              int pb_x, int pb_y, int pb_w, int pb_h, int pb_scup, int pb_idx, int cu_x, int cu_y, int cu_width, int cu_height, int bit_depth
#if MIPF
    , int mipf_enable_flag
#endif
)
{
    pel *rec, *pred_tb;
    int num_nz_temp[MAX_NUM_TB][N_C];
    int s_rec;
    int x_scu, y_scu, tb_x, tb_y, tb_w, tb_h, tb_scup;
    int pb_part_size = mod_info_curr->pb_part;
    u16 avail_cu;
    pel* resi_tb;
    int num_luma_in_prev_pb = 0;

    for (int i = 0; i < pb_idx; i++)
        num_luma_in_prev_pb += mod_info_curr->pb_info.sub_w[i] * mod_info_curr->pb_info.sub_h[i];

    int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_idx);
    int tb_idx_offset = get_tb_idx_offset(pb_part_size, pb_idx);
    get_tb_width_height_in_pb(pb_w, pb_h, pb_part_size, pb_idx, &tb_w, &tb_h);
    for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
    {
        get_tb_pos_in_pb(pb_x, pb_y, pb_part_size, tb_w, tb_h, tb_idx, &tb_x, &tb_y);
        x_scu = tb_x >> 2;
        y_scu = tb_y >> 2;
        tb_scup = y_scu * pic_width_in_scu + x_scu;

        if (tb_idx == 0)
            assert(tb_scup == pb_scup);

        avail_cu = com_get_avail_intra(x_scu, y_scu, pic_width_in_scu, tb_scup, map_scu);
        /* prediction */
        s_rec = pic->stride_luma;
        rec = pic->y + (tb_y * s_rec) + tb_x;
        com_get_nbr(tb_x, tb_y, tb_w, tb_h, rec, s_rec, avail_cu, nb, tb_scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, Y_C);

        pred_tb = mod_info_curr->pred[Y_C]; // pred is temp memory

#if SAWP
        if (mod_info_curr->sawp_flag)
        {
            pel pred0[MAX_CU_DIM] = { 0 };
            pel pred1[MAX_CU_DIM] = { 0 };
            com_ipred(nb[0][0] + STNUM, nb[0][1] + STNUM, pred0, mod_info_curr->sawp_idx0, tb_w, tb_h, bit_depth, avail_cu, mod_info_curr->ipf_flag
#if MIPF
                , mipf_enable_flag    
#endif
#if IIP
                , mod_info_curr->iip_flag
#endif
            );
            com_ipred(nb[0][0] + STNUM, nb[0][1] + STNUM, pred1, mod_info_curr->sawp_idx1, tb_w, tb_h, bit_depth, avail_cu, mod_info_curr->ipf_flag
#if MIPF
                , mipf_enable_flag   
#endif
#if IIP
                , mod_info_curr->iip_flag
#endif
            );
            pel awp_weight0[N_C][MAX_AWP_DIM];
            pel awp_weight1[N_C][MAX_AWP_DIM];
            /* derive weights */
#if BAWP
#if SAWP_SCC == 0
            com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1, 0, 1);
#else
            com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1, 0);
#endif
#else
            com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1);
#endif

            /* combine two pred buf */
            com_derive_sawp_pred(mod_info_curr, Y_C, pred0, pred1, awp_weight0[Y_C], awp_weight1[Y_C]);
        }
        else {
#endif // SAWP
        com_ipred(nb[0][0] + STNUM, nb[0][1] + STNUM, pred_tb, mod_info_curr->ipm[pb_idx][0], tb_w, tb_h, bit_depth, avail_cu, mod_info_curr->ipf_flag
#if MIPF
            , mipf_enable_flag
#endif
#if IIP
            , mod_info_curr->iip_flag
#endif
        );
#if SAWP
        }
#endif // SAWP
#if DT_INTRA_BOUNDARY_FILTER_OFF
        if (pb_part_size != SIZE_2Nx2N)
            assert(mod_info_curr->ipf_flag == 0);
#endif

        /* reconstruction */
        //get start of tb residual
        int coef_offset_tb = get_coef_offset_tb(cu_x, cu_y, tb_x, tb_y, cu_width, cu_height, mod_info_curr->tb_part);
        resi_tb = resi[Y_C] + coef_offset_tb; //residual is stored sequentially, not in raster
        assert((tb_w * tb_h) * tb_idx + num_luma_in_prev_pb == coef_offset_tb);
        //to fit the interface of com_recon()
        for( int comp = 0; comp < N_C; comp++ )
        {
            num_nz_temp[TB0][comp] = mod_info_curr->num_nz[tb_idx + tb_idx_offset][comp];
        }

        //here we treat a tb as one non-separable region; stride for resi_tb and pred are both tb_w; nnz shall be assigned to TB0
        com_recon(SIZE_2Nx2N, resi_tb, pred_tb, num_nz_temp, Y_C, tb_w, tb_h, s_rec, rec, bit_depth
#if SBT
            , 0
#endif
        );

        //update map
#if SAWP
        if (mod_info_curr->sawp_flag)
        {
            update_sawp_info_map_scu(mod_info_curr, map_scu, map_ipm, pic_width_in_scu);
        }
        else {
#endif // SAWP

            update_intra_info_map_scu(map_scu, map_ipm, tb_x, tb_y, tb_w, tb_h, pic_width_in_scu, mod_info_curr->ipm[pb_idx][0]);
#if SAWP
        }
#endif // SAWP

    }
}
#endif


void com_get_nbr(int x, int y, int width, int height, pel *src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 * map_scu, int pic_width_in_scu, int pic_height_in_scu, int bit_depth, int ch_type)
{
    int  i;
    int  width_in_scu  = (ch_type == Y_C) ? (width >> MIN_CU_LOG2)  : (width >> (MIN_CU_LOG2 - 1));
    int  height_in_scu = (ch_type == Y_C) ? (height >> MIN_CU_LOG2) : (height >> (MIN_CU_LOG2 - 1));
    int  unit_size = (ch_type == Y_C) ? MIN_CU_SIZE : (MIN_CU_SIZE >> 1);
    int  x_scu = PEL2SCU(ch_type == Y_C ? x : x << 1);
    int  y_scu = PEL2SCU(ch_type == Y_C ? y : y << 1);
    pel *const src_bak = src;
    pel *left = nb[ch_type][0] + STNUM;
    pel *up   = nb[ch_type][1] + STNUM;
    int pad_le = height;  //number of padding pixel in the left column
    int pad_up = width;   //number of padding pixel in the upper row
    int pad_le_in_scu = height_in_scu;
    int pad_up_in_scu = width_in_scu;

    com_mset_16b(left - STNUM, 1 << (bit_depth - 1), height + pad_le + STNUM);
    com_mset_16b(up   - STNUM, 1 << (bit_depth - 1), width  + pad_up + STNUM);
    if(IS_AVAIL(avail_cu, AVAIL_UP))
    {
        com_mcpy(up, src - s_src, width * sizeof(pel));
        for(i = 0; i < pad_up_in_scu; i++)
        {
            if(x_scu + width_in_scu + i < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[scup - pic_width_in_scu + width_in_scu + i]))
            {
                com_mcpy(up + width + i * unit_size, src - s_src + width + i * unit_size, unit_size * sizeof(pel));
            }
            else
            {
                com_mset_16b(up + width + i * unit_size, up[width + i * unit_size - 1], unit_size);
            }
        }
    }

    if(IS_AVAIL(avail_cu, AVAIL_LE))
    {
        src--;
        for(i = 0; i < height; ++i)
        {
            left[i] = *src;
            src += s_src;
        }
        for(i = 0; i < pad_le_in_scu; i++)
        {
            if(y_scu + height_in_scu + i < pic_height_in_scu && MCU_GET_CODED_FLAG(map_scu[scup - 1 + (height_in_scu + i) *pic_width_in_scu]))
            {
                int j;
                for(j = 0; j < unit_size; ++j)
                {
                    left[height + i * unit_size + j] = *src;
                    src += s_src;
                }
            }
            else
            {
                com_mset_16b(left + height + i * unit_size, left[height + i * unit_size - 1], unit_size);
                src += (s_src * unit_size);
            }
        }
    }

    if (IS_AVAIL(avail_cu, AVAIL_UP_LE))
    {
        up[-1] = left[-1] = src_bak[-s_src - 1];
    }
    else if (IS_AVAIL(avail_cu, AVAIL_UP))
    {
        up[-1] = left[-1] = up[0];
    }
    else if (IS_AVAIL(avail_cu, AVAIL_LE))
    {
        up[-1] = left[-1] = left[0];
    }

    up[-2] = left[0];
    left[-2] = up[0];
    up[-3] = left[1];
    left[-3] = up[1];
#if IIP
    if (STNUM > 3)
    {
        up[-4] = left[2];
        left[-4] = up[2];
        up[-5] = left[3];
        left[-5] = up[3];
    }
#endif
}

void ipred_hor(pel *src_le, pel *dst, int w, int h
#if IIP
    , int iip_flag
#endif
)
{
#if IIP
    int bit_shift_v = 8;
#endif
    int i, j;
    {
        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
#if IIP
                if (iip_flag)
                {
                    dst[j] = (s16)((src_le[0] * sfcoeff_1D_v[2] + (src_le[-1] + src_le[1])*sfcoeff_1D_v[1] + (src_le[-2] + src_le[2])*sfcoeff_1D_v[0]) >> bit_shift_v);
                }
                else
                {
#endif
                    dst[j] = src_le[0];
#if IIP
                }
#endif
            }
            dst += w;
            src_le++;
        }
    }
}

void ipred_vert(pel *src_up, pel *dst, int w, int h
#if IIP
    , int iip_flag
#endif
)
{
#if IIP
    int bit_shift_h = 8;
#endif
    int i, j;
    for(i = 0; i < h; i++)
    {
        for(j = 0; j < w; j++)
        {
#if IIP
            if (iip_flag)
            {
                dst[j] = (s16)((src_up[j] * sfcoeff_1D_h[2] + (src_up[j - 1] + src_up[j + 1])*sfcoeff_1D_h[1] + (src_up[j - 2] + src_up[j + 2])*sfcoeff_1D_h[0]) >> bit_shift_h);
            }
            else
            {
#endif
                dst[j] = src_up[j];
#if IIP
            }
#endif
        }
        dst += w;
    }
}

void ipred_dc(pel *src_le, pel *src_up, pel *dst, int w, int h, int bit_depth, u16 avail_cu
#if IIP
    , int iip_flag
#endif
)
{
    assert(com_tbl_log2[w] >= 2);
    assert(com_tbl_log2[h] >= 2);
#if IIP
    const s32 filter1[4] = { 18,38,18,182 };
    const s32 filter2[2] = { 13,243 };
    const s32 filter3[3] = { 13,13,230 };
    const s32 filter4[4] = { 38,18,38,162 };
    const s32  fshift = 8;
#endif
    int dc = 0;
    int wh, i, j;
    if(IS_AVAIL(avail_cu, AVAIL_LE))
    {
        for (i = 0; i < h; i++)
        {
            dc += src_le[i];
        }
        if(IS_AVAIL(avail_cu, AVAIL_UP))
        {
            for (j = 0; j < w; j++)
            {
                dc += src_up[j];
            }
            dc = (dc + ((w + h) >> 1)) * (4096 / (w + h)) >> 12;
        }
        else
        {
            dc = (dc + (h >> 1)) >> com_tbl_log2[h];
        }
    }
    else if(IS_AVAIL(avail_cu, AVAIL_UP))
    {
        for (j = 0; j < w; j++)
        {
            dc += src_up[j];
        }
        dc = (dc + (w >> 1)) >> com_tbl_log2[w];
    }
    else
    {
        dc = 1 << (bit_depth - 1);
    }
#if IIP
    if (iip_flag)
    {
        const int dcf13 = dc*filter1[3];
        const int dcf21 = dc*filter2[1];

        for (int i = 1; i < w; i++)
        {
            dst[i] = (src_up[i - 1] * filter1[0] + src_up[i] * filter1[1] + src_up[i + 1] * filter1[2] + dcf13 + (1 << (fshift - 1))) >> fshift;
            dst[w + i] = (src_up[i] * filter2[0] + dcf21 + (1 << (fshift - 1))) >> fshift;
        }
        for (int i = 1; i < h; i++)
        {
            dst[w*i] = (src_le[i - 1] * filter1[0] + src_le[i] * filter1[1] + src_le[i + 1] * filter1[2] + dcf13 + (1 << (fshift - 1))) >> fshift;
            dst[w*i + 1] = (src_le[i] * filter2[0] + dcf21 + (1 << (fshift - 1))) >> fshift;
        }
        dst[0] = (src_up[0] * filter4[0] + src_up[-1] * filter4[1] + src_le[0] * filter4[2] + (src_up[1]  + src_le[1] -2*dc)* 18 + dc*filter4[3]  + (1 << (fshift - 1))) >> fshift;
        dst[1] = (src_up[0] * filter1[0] + src_up[1] * filter1[1] + src_up[2] * filter1[2] + (src_le[0]-dc) * 13 + dcf13 + (1 << (fshift - 1))) >> fshift;
        dst[w] = (src_le[0] * filter1[0] + src_le[1] * filter1[1] + src_le[2] * filter1[2] + (src_up[0]-dc) * 13 + dcf13 + (1 << (fshift - 1))) >> fshift;
        dst[w + 1] = ((src_up[1] + src_le[1])*filter3[0] + dc*filter3[2] + (1 << (fshift - 1))) >> fshift;
        for (int j = 2; j < h; j++)
        {
            for (int i = 2; i < w; i++)
            {
                dst[w*j + i] = (pel)dc;
            }
        }
    }
    else
    {
#endif
    wh = w * h;
    for(i = 0; i < wh; i++)
    {
        dst[i] = (pel)dc;
    }
#if IIP
    }
#endif
}

void ipred_plane(pel *src_le, pel *src_up, pel *dst, int w, int h
#if IIP
    , int iip_flag
#endif
)
{
    assert(com_tbl_log2[w] >= 2);
    assert(com_tbl_log2[h] >= 2);

    pel *rsrc;
    int  coef_h = 0, coef_v = 0;
    int  a, b, c, x, y;
    int  w2 = w >> 1;
    int  h2 = h >> 1;
    int  ib_mult[5]  = { 13, 17, 5, 11, 23 };
    int  ib_shift[5] = { 7, 10, 11, 15, 19 };
    int  idx_w = com_tbl_log2[w] - 2;
    int  idx_h = com_tbl_log2[h] - 2;
    int  im_h, is_h, im_v, is_v, temp, temp2;
    im_h = ib_mult[idx_w];
    is_h = ib_shift[idx_w];
    im_v = ib_mult[idx_h];
    is_v = ib_shift[idx_h];
    rsrc = src_up + (w2 - 1);
#if IIP
    const s32 filter1[4] = { 18,38,18,182 };
    const s32 filter2[2] = { 13,243 };
    const s32 filter3[3] = { 13,13,230 };
    const s32 filter4[4] = { 38,18,38,162 };
    const s8  fshift = 8;
    int base;
    pel *dst_f = dst;
#endif
    for (x = 1; x < w2 + 1; x++)
    {
        coef_h += x * (rsrc[x] - rsrc[-x]);
    }
    rsrc = src_le + (h2 - 1);
    for (y = 1; y < h2 + 1; y++)
    {
        coef_v += y * (rsrc[y] - rsrc[-y]);
    }
    a = (src_le[h - 1] + src_up[w - 1]) << 4;
    b = ((coef_h << 5) * im_h + (1 << (is_h - 1))) >> is_h;
    c = ((coef_v << 5) * im_v + (1 << (is_v - 1))) >> is_v;
    temp = a - (h2 - 1) * c - (w2 - 1) * b + 16;
    for (y = 0; y < h; y++)
    {
        temp2 = temp;
        for (x = 0; x < w; x++)
        {
            dst[x] = (pel)(temp2 >> 5);
            temp2 += b;
        }
        temp += c;
        dst += w;
    }
#if IIP
    if (iip_flag)
    {
        base = a - (h2 - 1) * c - (w2 - 1) * b + 16;
        int basef13 = base*filter1[3];
        int basef21 = base*filter2[1];
        const int bf13 = b*filter1[3];
        const int bf21 = b*filter2[1];
        const int cf13 = c*filter1[3];
        const int cf21 = c*filter2[1];
        for (int i = 1; i < w; i++)
        {
            basef13 += bf13;
            basef21 += bf21;
            dst_f[i] = (src_up[i - 1]  * filter1[0] + src_up[i] * filter1[1] + src_up[i + 1] * filter1[2] + ((basef13+87*c)>>5)  + (1 << (fshift - 1))) >> fshift;
            dst_f[w + i] = (src_up[i] * filter2[0] + ((basef21 +269 * c)>>5) + (1 << (fshift - 1))) >> fshift;

        }
        basef13 = base*filter1[3];
        basef21 = base*filter2[1];
        for (int i = 1; i < h; i++)
        { 
            basef13 += cf13;
            basef21 += cf21;
            dst_f[w*i] = (src_le[i - 1] * filter1[0] + src_le[i] * filter1[1] + src_le[i + 1] * filter1[2] + ((basef13+87*b)>>5)  + (1 << (fshift -1))) >> fshift ;
            dst_f[w*i + 1] = (src_le[i] * filter2[0] + ((basef21+ 269 * b)>>5) + (1 << (fshift -1))) >> fshift;
        }
        dst_f[0] = (src_up[0] * filter4[0] + src_up[-1] * filter4[1] + src_le[0] * filter4[2] + (src_up[1]  + src_le[1])*18 + ((-36 * base + base*filter4[3] + 69 * b + 69*c)>>5) + (1 << (fshift -1))) >> fshift;
        dst_f[1] = (src_up[0] * filter1[0] + src_up[1] * filter1[1] + src_up[2] * filter1[2] + src_le[0] * 13 + ((-base * 13 + base*filter1[3] + 195 * b + 87 * c)>>5) + (1 << (fshift -1))) >> fshift ;
        dst_f[w] = (src_le[0] * filter1[0] + src_le[1] * filter1[1] + src_le[2] * filter1[2] + src_up[0] * 13 + ((-base * 13 + base*filter1[3] + 195 * c + 87 * b)>>5) + (1 << (fshift -1))) >> fshift;
        dst_f[w + 1] = ((src_up[1] + src_le[1])*filter3[0] + ((base*filter3[2] + (b + c) * 256)>>5) + (1 << (fshift -1))) >> fshift ;
    }
#endif
}

void ipred_bi(pel *src_le, pel *src_up, pel *dst, int w, int h
#if IIP
    , int iip_flag
#endif
)
{
    assert(com_tbl_log2[w] >= 2);
    assert(com_tbl_log2[h] >= 2);

    int x, y;
    int ishift_x = com_tbl_log2[w];
    int ishift_y = com_tbl_log2[h];
    int ishift = COM_MIN(ishift_x, ishift_y);
    int ishift_xy = ishift_x + ishift_y + 1;
    int offset = 1 << (ishift_x + ishift_y);
    int a, b, c, wt, wxy, tmp;
    int predx;
#if IIP
    int ref_up_buf[MAX_CU_SIZE + 4], ref_le_buf[MAX_CU_SIZE + 4], up_buf[MAX_CU_SIZE + 4], le_buf[MAX_CU_SIZE + 4];
    int *ref_up_p, *ref_le_p, *up_p, *le_p;
    ref_up_p = ref_up_buf + 2;
    ref_le_p = ref_le_buf + 2;
    up_p = up_buf + 2;
    le_p = le_buf + 2;
    const s32 filter[5] = { 82,252,356,252,82 };
    const s32 filter2D[6] = { 7,  20,  28,  62,  88,  124 };
    const int fshift = 10;
    pel* dst_f = dst;
    int ref_up[MAX_CU_SIZE], ref_le[MAX_CU_SIZE], up[MAX_CU_SIZE], le[MAX_CU_SIZE], wy[MAX_CU_SIZE + 2];
#else
    int ref_up[MAX_CU_SIZE], ref_le[MAX_CU_SIZE], up[MAX_CU_SIZE], le[MAX_CU_SIZE], wy[MAX_CU_SIZE];
#endif
    int wc, tbl_wc[6] = {-1, 21, 13, 7, 4, 2};
    wc = ishift_x > ishift_y ? ishift_x - ishift_y : ishift_y - ishift_x;
    com_assert(wc <= 5);

    wc = tbl_wc[wc];
    for( x = 0; x < w; x++ )
    {
        ref_up[x] = src_up[x];
    }
    for( y = 0; y < h; y++ )
    {
        ref_le[y] = src_le[y];
    }

    a = src_up[w - 1];
    b = src_le[h - 1];
    c = (w == h) ? (a + b + 1) >> 1 : (((a << ishift_x) + (b << ishift_y)) * wc + (1 << (ishift + 5))) >> (ishift + 6);
    wt = (c << 1) - a - b;
    for( x = 0; x < w; x++ )
    {
        up[x] = b - ref_up[x];
        ref_up[x] <<= ishift_y;
    }
    tmp = 0;
    for( y = 0; y < h; y++ )
    {
        le[y] = a - ref_le[y];
        ref_le[y] <<= ishift_x;
        wy[y] = tmp;
        tmp += wt;
    }
#if IIP
    if (iip_flag)
    {
        wy[h] = wy[h - 1] + wt;
        wy[h + 1] = wy[h] + wt;

        up_p[-1] = b - src_up[-1];
        up_p[-2] = b - src_up[-2];
        ref_up_p[-1] = src_up[-1] << ishift_y;
        ref_up_p[-2] = src_up[-2] << ishift_y;
        for (x = 0; x < w; x++)
        {
            up_p[x] = up[x];
            ref_up_p[x] = ref_up[x];
        }
        up_p[w] = b - src_up[w];
        up_p[w + 1] = b - src_up[w + 1];
        ref_up_p[w] = src_up[w] << ishift_y;
        ref_up_p[w + 1] = src_up[w + 1] << ishift_y;
        for (x = 0; x < w; x++)
        {
            up[x] = (up_p[x - 2] + up_p[x + 2])*filter[0] + (up_p[x - 1] + up_p[x + 1])*filter[1] + up_p[x] * filter[2];
            ref_up[x] = (ref_up_p[x - 2] + ref_up_p[x + 2])*filter[0] + (ref_up_p[x - 1] + ref_up_p[x + 1])*filter[1] + ref_up_p[x] * filter[2];
        }
        ////////////////////////
        le_p[-1] = a - src_le[-1];
        le_p[-2] = a - src_le[-2];
        ref_le_p[-1] = src_le[-1] << ishift_x;
        ref_le_p[-2] = src_le[-2] << ishift_x;
        for (y = 0; y < h; y++)
        {
            le_p[y] = le[y];
            ref_le_p[y] = ref_le[y];
        }
        le_p[h] = a - src_le[h];
        le_p[h + 1] = a - src_le[h + 1];
        ref_le_p[h] = src_le[h] << ishift_x;
        ref_le_p[h + 1] = src_le[h + 1] << ishift_x;
        for (y = 0; y < h; y++)
        {
            le[y] = (le_p[y - 2] + le_p[y + 2])*filter[0] + (le_p[y - 1] + le_p[y + 1])*filter[1] + le_p[y] * filter[2];
            ref_le[y] = (ref_le_p[y - 2] + ref_le_p[y + 2])*filter[0] + (ref_le_p[y - 1] + ref_le_p[y + 1])*filter[1] + ref_le_p[y] * filter[2];
        }
    }
#endif
    for( y = 0; y < h; y++ )
    {
        predx = ref_le[y];
        wxy = 0;
        for (x = 0; x < w; x++)
        {
            predx += le[y];
            ref_up[x] += up[x];
#if IIP
            if (iip_flag)
            {
                dst[x] = (pel)((((s64)predx << ishift_y) + ((s64)ref_up[x] << ishift_x) + ((s64)wxy<< fshift) + ((s64)1 << (ishift_xy + fshift - 1))) >> ((s64)ishift_xy + fshift));
                wxy += wy[y];
                
            }
            else
            {
#endif  
                dst[x] = (pel)(((predx << ishift_y) + (ref_up[x] << ishift_x) + wxy + offset) >> ishift_xy);
                wxy += wy[y];
#if IIP
            }
#endif
        }
        dst += w;
    }
}

#if PMC || EPMC
void subtract_sample_for_tscpm(int width_c, int height_c, int bit_depth
    , pel *p_src0, int stride_src0
    , pel *p_src1, int stride_src1
    , pel *p_dst,  int stride_dst
#if EPMC
    , int pmc_type
#endif
)
{
    int max_val = (1 << bit_depth) - 1;
    for (int j = 0; j < height_c; j++)
    {
        for (int i = 0; i < width_c; i++)
        {
#if EPMC
            if (pmc_type == 1)
            {
                p_dst[i + j * stride_dst] = COM_CLIP3(0, max_val, (p_src0[i + j * stride_src0] - (p_src1[i + j * stride_src1] + 1) / 2));
            }
            else if (pmc_type == 2)
            {
                p_dst[i + j * stride_dst] = COM_CLIP3(0, max_val, (p_src0[i + j * stride_src0] - p_src1[i + j * stride_src1] * 2));          
            }
            else
            {
#endif
                p_dst[i + j * stride_dst]  = COM_CLIP3(0, max_val, (p_src0[i + j * stride_src0] - p_src1[i + j * stride_src1]));
#if EPMC
            }
#endif
        }
    }
}
#endif

#if TSCPM
void downsample_pixel_for_tscpm(int width_c, int height_c, int bit_depth,
                          pel *p_src, int stride_src,
                          pel *p_dst, int stride_dst)
{
    int max_val = (1 << bit_depth) - 1;
    int temp_val;

    for (int j = 0; j < height_c; j++)
    {
        for (int i = 0; i < width_c; i++)
        {
            if (i == 0)
            {
                temp_val = (p_src[2 * i] + p_src[2 * i + stride_src] + 1) >> 1;
            }
            else
            {
                temp_val = (p_src[2 * i] * 2 + p_src[2 * i + 1] + p_src[2 * i - 1]
                             + p_src[2 * i + stride_src] * 2
                             + p_src[2 * i + stride_src + 1]
                             + p_src[2 * i + stride_src - 1]
                             + 4) >> 3;
            }

#if !PMC && !EPMC
            if (temp_val > max_val || temp_val < 0)
            {
                printf("\n TSCPM clip error");
            }
#endif
            p_dst[i] = temp_val;
        }
        p_dst += stride_dst;
        p_src += stride_src * 2;
    }
}

pel get_luma_border_pixel_for_tscpm(int idx, int b_above_pixel, int width_c, int height_c, int is_above, int is_left, pel nb[N_C][N_REF][MAX_CU_SIZE * 3])
{
    pel *p_src = NULL;
    pel dst_pixel = -1;
    // Simplify Version, only copy rec luma.
    if (b_above_pixel)
    {
        if (is_above)
        {
            p_src = nb[Y_C][1] + STNUM;
            int i = idx;
            if (i < width_c)
            {
                if (i == 0 && !is_left)
                {
                    dst_pixel = (3 * p_src[2 * i] + p_src[2 * i + 1] + 2) >> 2;
                }
                else
                {
                    dst_pixel = (2 * p_src[2 * i] + p_src[2 * i - 1] + p_src[2 * i + 1] + 2) >> 2;
                }
            }
        }
    }
    else
    {
        if (is_left)
        {
            p_src = nb[Y_C][0] + STNUM;
            int j = idx;
            if (j < height_c)
            {
                dst_pixel = (p_src[2 * j] + p_src[2 * j  + 1] + 1) >> 1;
            }
        }
    }

    if (dst_pixel < 0)
    {
        printf("\n Error get dstPoint in xGetLumaBorderPixel");
    }
    return dst_pixel;
}

#define GET_SRC_PIXEL(idx, b_above_pixel)  get_luma_border_pixel_for_tscpm((idx), (b_above_pixel), width_c, height_c, is_above, is_left, nb)
#define SWAP_PIXEL(a, b, type)            {type swap_temp; swap_temp = (a); (a) = (b); (b) = swap_temp;}

void get_linear_param_for_tscpm(int comp_id, int *a, int *b, int *shift, int is_above, int is_left, int width_c, int height_c, int bit_depth, pel nb[N_C][N_REF][MAX_CU_SIZE * 3]
#if ENHANCE_TSPCM || PMC || EPMC
    , int ipm_c
#endif
)
{
    pel *p_cur = NULL;
    pel *p_cur_left  = NULL;
    pel *p_cur_above = NULL;
    p_cur_left  = nb[comp_id][0] + STNUM;
    p_cur_above = nb[comp_id][1] + STNUM;

    int min_dim = is_left && is_above ? min(height_c, width_c) : (is_left ? height_c : width_c);
    int num_steps = min_dim;
    int y_max = 0;
    int x_max = -MAX_INT;
    int y_min = 0;
    int x_min = MAX_INT;

    // four points start
    int ref_pixel_luma[4]   = { -1, -1, -1, -1 };
    int ref_pixel_chroma[4] = { -1, -1, -1, -1 };

#if ENHANCE_TSPCM || PMC || EPMC
    switch (ipm_c)
    {
    case IPD_TSCPM_C:
#if PMC
    case IPD_MCPM_C:
#endif
#if EPMC
    case IPD_EMCPM_C:
    case IPD_EMCPM2_C:
#endif
#endif
    if (is_above)
    {
        p_cur = p_cur_above;
        int idx = ((num_steps - 1) * width_c) / min_dim;
        ref_pixel_luma[0]   = GET_SRC_PIXEL(0,   1); // pSrc[0];
        ref_pixel_chroma[0] = p_cur[0];
        ref_pixel_luma[1]   = GET_SRC_PIXEL(idx, 1); // pSrc[idx];
        ref_pixel_chroma[1] = p_cur[idx];
        // using 4 points when only one border
        if (!is_left && width_c >= 4)
        {
            int step = width_c >> 2;
            for (int i = 0; i < 4; i++)
            {
                ref_pixel_luma[i]   = GET_SRC_PIXEL(i * step, 1); // pSrc[i * uiStep];
                ref_pixel_chroma[i] = p_cur[i * step];
            }
        }
    }
    if (is_left)
    {
        p_cur = p_cur_left;
        int idx = ((num_steps - 1) * height_c) / min_dim;
        ref_pixel_luma[2]   = GET_SRC_PIXEL(0,   0); // pSrc[0];
        ref_pixel_chroma[2] = p_cur[0];
        ref_pixel_luma[3]   = GET_SRC_PIXEL(idx, 0); // pSrc[idx * iSrcStride];
        ref_pixel_chroma[3] = p_cur[idx];
        // using 4 points when only one border
        if (!is_above && height_c >= 4)
        {
            int step = height_c >> 2;
            for (int i = 0; i < 4; i++)
            {
                ref_pixel_luma[i]   = GET_SRC_PIXEL(i * step, 0); // pSrc[i * step * iSrcStride];
                ref_pixel_chroma[i] = p_cur[i * step];
            }
        }
    }
#if ENHANCE_TSPCM || PMC || EPMC
        break;
#if ENHANCE_LT_MODE
#if ENHANCE_TSPCM
    case IPD_TSCPM_LT_C:
#endif
#if PMC
    case IPD_MCPM_LT_C:
#endif
#if EPMC
    case IPD_EMCPM_LT_C:
    case IPD_EMCPM2_LT_C:
#endif
        if (is_above)
        {
            p_cur = p_cur_above;
            int idx1 = (3 * width_c) >> 3;
            int idx2 = (5 * width_c) >> 3;
            ref_pixel_luma[0] = GET_SRC_PIXEL(idx1, 1);
            ref_pixel_chroma[0] = p_cur[idx1];
            ref_pixel_luma[1] = GET_SRC_PIXEL(idx2, 1);
            ref_pixel_chroma[1] = p_cur[idx2];
            // using 4 points when only one border
            if (!is_left && width_c >= 4)
            {
                int step = width_c >> 2;
                int start = step >> 1;
                for (int i = 0; i < 4; i++)
                {
                    ref_pixel_luma[i] = GET_SRC_PIXEL(i * step + start, 1);
                    ref_pixel_chroma[i] = p_cur[i * step + start];
                }
            }
        }
        if (is_left)
        {
            p_cur = p_cur_left;
            int idx1 = (3 * height_c) >> 3;
            int idx2 = (5 * height_c) >> 3;
            ref_pixel_luma[2] = GET_SRC_PIXEL(idx1, 0);
            ref_pixel_chroma[2] = p_cur[idx1];
            ref_pixel_luma[3] = GET_SRC_PIXEL(idx2, 0);
            ref_pixel_chroma[3] = p_cur[idx2];
            // using 4 points when only one border
            if (!is_above && height_c >= 4)
            {
                int step = height_c >> 2;
                int start = step >> 1;
                for (int i = 0; i < 4; i++)
                {
                    ref_pixel_luma[i] = GET_SRC_PIXEL(i * step + start, 0);
                    ref_pixel_chroma[i] = p_cur[i * step + start];
                }
            }
        }
        break;
#endif
#if ENHANCE_TSPCM
    case IPD_TSCPM_T_C:
#endif
#if PMC
    case IPD_MCPM_T_C:
#endif
#if EPMC
    case IPD_EMCPM_T_C:
    case IPD_EMCPM2_T_C:
#endif
#if ENHANCE_TSCPM_BUGFIX
        is_left = 0;
#endif
        // top reference
        if (is_above) 
        {
            p_cur = p_cur_above;
            // using 4 points when only one border
            if (width_c >= 4) 
            {
                int step = width_c >> 2;
                for (int i = 0; i < 4; i++) 
                {
                    ref_pixel_luma[i]   = GET_SRC_PIXEL(i * step, 1); // pSrc[i * step];
                    ref_pixel_chroma[i] = p_cur[i * step];
                }
            }
        }
        break;
#if ENHANCE_TSPCM
    case IPD_TSCPM_L_C:
#endif
#if PMC
    case IPD_MCPM_L_C:
#endif
#if EPMC
    case IPD_EMCPM_L_C:
    case IPD_EMCPM2_L_C:
#endif
#if ENHANCE_TSCPM_BUGFIX
        is_above = 0;
#endif
        // left reference
        if (is_left) 
        {
            p_cur = p_cur_left;
            // using 4 points when only one border
            if (height_c >= 4) 
            {
                int step = height_c >> 2;
                for (int i = 0; i < 4; i++) 
                {
                    ref_pixel_luma[i]   = GET_SRC_PIXEL(i * step, 0); // pSrc[i * step * iSrcStride];
                    ref_pixel_chroma[i] = p_cur[i * step];
                }
            }
        }
        break;
    default:
        printf("\n illegal TSCPM prediction mode\n");
        assert(0);
        break;
    }
#endif
    if (   (is_above &&  is_left)
        || (is_above && !is_left  && width_c  >= 4)
        || (is_left  && !is_above && height_c >= 4) )
    {
        int min_grp_idx[2] = { 0, 2 };
        int max_grp_idx[2] = { 1, 3 };
        int *tmp_min_grp = min_grp_idx;
        int *tmp_max_grp = max_grp_idx;
        if (ref_pixel_luma[tmp_min_grp[0]] > ref_pixel_luma[tmp_min_grp[1]]) SWAP_PIXEL(tmp_min_grp[0], tmp_min_grp[1], int);
        if (ref_pixel_luma[tmp_max_grp[0]] > ref_pixel_luma[tmp_max_grp[1]]) SWAP_PIXEL(tmp_max_grp[0], tmp_max_grp[1], int);
        if (ref_pixel_luma[tmp_min_grp[0]] > ref_pixel_luma[tmp_max_grp[1]]) SWAP_PIXEL(tmp_min_grp,    tmp_max_grp,    int *);
        if (ref_pixel_luma[tmp_min_grp[1]] > ref_pixel_luma[tmp_max_grp[0]]) SWAP_PIXEL(tmp_min_grp[1], tmp_max_grp[0], int);

        assert(ref_pixel_luma[tmp_max_grp[0]] >= ref_pixel_luma[tmp_min_grp[0]]);
        assert(ref_pixel_luma[tmp_max_grp[0]] >= ref_pixel_luma[tmp_min_grp[1]]);
        assert(ref_pixel_luma[tmp_max_grp[1]] >= ref_pixel_luma[tmp_min_grp[0]]);
        assert(ref_pixel_luma[tmp_max_grp[1]] >= ref_pixel_luma[tmp_min_grp[1]]);

        x_min = (ref_pixel_luma  [tmp_min_grp[0]] + ref_pixel_luma  [tmp_min_grp[1]] + 1 )>> 1;
        y_min = (ref_pixel_chroma[tmp_min_grp[0]] + ref_pixel_chroma[tmp_min_grp[1]] + 1) >> 1;
        
        x_max = (ref_pixel_luma  [tmp_max_grp[0]] + ref_pixel_luma  [tmp_max_grp[1]] + 1 )>> 1;
        y_max = (ref_pixel_chroma[tmp_max_grp[0]] + ref_pixel_chroma[tmp_max_grp[1]] + 1) >> 1;
    }
    else if (is_above)
    {
        for (int k = 0; k < 2; k++)
        {
            if (ref_pixel_luma[k] > x_max)
            {
                x_max = ref_pixel_luma[k];
                y_max = ref_pixel_chroma[k];
            }
            if (ref_pixel_luma[k] < x_min)
            {
                x_min = ref_pixel_luma[k];
                y_min = ref_pixel_chroma[k];
            }
        }
    }
    else if (is_left)
    {
        for (int k = 2; k < 4; k++)
        {
            if (ref_pixel_luma[k] > x_max)
            {
                x_max = ref_pixel_luma[k];
                y_max = ref_pixel_chroma[k];
            }
            if (ref_pixel_luma[k] < x_min)
            {
                x_min = ref_pixel_luma[k];
                y_min = ref_pixel_chroma[k];
            }
        }
    }
    // four points end

    if (is_left || is_above)
    {
        *a = 0;
        *shift = 16;
        int diff = x_max - x_min;
        int add = 0;
        int shift_div = 0;
        if (diff > 64)
        {
            shift_div = (bit_depth > 8) ? bit_depth - 6 : 2;
            add = shift_div ? 1 << (shift_div - 1) : 0;
            diff = (diff + add) >> shift_div;

            if (bit_depth == 10)
            {
                assert(shift_div == 4 && add == 8); // for default 10bit
            }
        }

        if (diff > 0)
        {
            *a = ((y_max - y_min) * g_aiTscpmDivTable64[diff - 1] + add) >> shift_div;
        }
        *b = y_min - (((s64)(*a) * x_min) >> (*shift));
    }
    if (!is_left && !is_above)
    {
        *a = 0;
        *b = 1 << (bit_depth - 1);
        *shift = 0;
        return;
    }
}

void linear_transform_for_tscpm(pel *p_src, int stride_src, pel *p_dst, int stride_dst, int a, int shift, int b, int width, int height, int bit_depth)
{
    int max_val = (1 << bit_depth) - 1;

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            int temp_val = (((s64)a * p_src[i]) >> (shift >= 0 ? shift : 0)) + b;
            p_dst[i] = COM_CLIP3(0, max_val, temp_val);
        }
        p_dst += stride_dst;
        p_src += stride_src;
    }
}

void ipred_tscpm(int comp_id, pel *pred_uv, pel *reco_y, int stride_y,int width, int height, int is_above, int is_left, int bit_depth, pel nb[N_C][N_REF][MAX_CU_SIZE * 3]
#if ENHANCE_TSPCM || PMC
    , int ipm_c
#endif
)
{
    int a, b, shift;   // parameters of Linear Model : a, b, shift
    get_linear_param_for_tscpm(comp_id, &a, &b, &shift, is_above, is_left, width, height, bit_depth, nb
#if ENHANCE_TSPCM || PMC || EPMC
        , ipm_c
#endif
    );

    pel pred_linear[MAX_CU_SIZE * MAX_CU_SIZE];
    int stride_linear = MAX_CU_SIZE;
    linear_transform_for_tscpm(reco_y, stride_y, pred_linear, stride_linear,
                         a, shift, b, (width << 1), (height << 1), bit_depth);
    int stride_uv = width;
    downsample_pixel_for_tscpm(width, height, bit_depth, pred_linear, stride_linear, pred_uv, stride_uv);
}

#if PMC || EPMC
void ipred_mcpm(int comp_id, pel *pred_uv, pel *reco_y, int stride_y,int width, int height, int is_above, int is_left, int bit_depth, pel nb[N_C][N_REF][MAX_CU_SIZE * 3]
    , int ipm_c
#if EPMC
    , int pmc_type
#endif
    , pel* reco_u, int stride_u
)
{
#if PMC
    int mcpm_flag = com_is_mcpm(ipm_c);
#endif
#if EPMC
    int emcpm_flag = com_is_emcpm(ipm_c);
    if (pmc_type == 0)
    {
#endif
#if PMC
        assert(mcpm_flag);
#endif
#if EPMC
    }
    else if (pmc_type !=0)
    {
        assert(emcpm_flag);
    }
#endif

    if (comp_id == U_C)
    {
        ipred_tscpm(comp_id, pred_uv, reco_y, stride_y, width, height, is_above, is_left, bit_depth, nb
            , ipm_c
        );
        return;
    }


    int a_v, b_v, shift_v;
    get_linear_param_for_tscpm(V_C, &a_v, &b_v, &shift_v, is_above, is_left, width, height, bit_depth, nb
        , ipm_c
    );
    int a_u, b_u, shift_u;
    get_linear_param_for_tscpm(U_C, &a_u, &b_u, &shift_u, is_above, is_left, width, height, bit_depth, nb
        , ipm_c
    );
    assert(shift_v == shift_u);
#if EPMC
    if (pmc_type == 0)
    {
#endif
        a_v = a_v + a_u;
        b_v = b_v + b_u;
#if EPMC
    }
    else if (pmc_type == MOD_IDX)
    {
        a_v = a_v + (a_u + 1) / 2;
        b_v = b_v + (b_u + 1) / 2;
    }
    else if (pmc_type == MOD2_IDX)
    {
        a_v = a_v + 2 * a_u;
        b_v = b_v + 2 * b_u;
    }
    else
    {
        assert(0);
    }
#endif
    pel pred_linear[MAX_CU_SIZE * MAX_CU_SIZE];
    int stride_linear = MAX_CU_SIZE;
    linear_transform_for_tscpm(reco_y, stride_y, pred_linear, stride_linear,
                         a_v, shift_v, b_v, (width << 1), (height << 1)
#if MOD_IDX >1 || MOD2_IDX >1
                         , bit_depth + 2
#else
                         , bit_depth + 1
#endif

    );

    int stride_uv = width;
    downsample_pixel_for_tscpm(width, height, bit_depth, pred_linear, stride_linear, pred_uv, stride_uv);

    subtract_sample_for_tscpm(width, height, bit_depth
        , pred_uv, stride_uv, reco_u, stride_u, pred_uv, stride_uv
#if EPMC
        , pmc_type
#endif
    );

}
#endif

#endif

#define GET_REF_POS(mt,d_in,d_out,offset) \
    (d_out) = ((d_in) * (mt)) >> 10;\
    (offset) = ((((d_in) * (mt)) << 5) >> 10) - ((d_out) << 5);

#define ADI_4T_FILTER_BITS                 7
#define ADI_4T_FILTER_OFFSET              (1<<(ADI_4T_FILTER_BITS-1))

void ipred_ang(pel *src_le, pel *src_up, pel *dst, int w, int h, int ipm
#if MIPF
    , int is_luma, int mipf_enable_flag
#endif
#if IIP
    , int iip_flag
#endif
)
{
#if MIPF
    const s16(*tbl_filt_list[4])[4] = { com_tbl_ipred_adi + 32, com_tbl_ipred_adi + 64, tbl_mc_c_coeff_hp, com_tbl_ipred_adi};
    const int filter_bits_list[4] = { 7, 7, 6, 7 };
    const int filter_offset_list[4] = { 64, 64 ,32, 64 };
    const int is_small = w * h <= (is_luma ? MIPF_TH_SIZE : MIPF_TH_SIZE_CHROMA);
    const int td = is_luma ? MIPF_TH_DIST : MIPF_TH_DIST_CHROMA;
    int filter_idx;
#else
    const s16(*tbl_filt)[4] = com_tbl_ipred_adi;
    const int filter_offset = ADI_4T_FILTER_OFFSET;
    const int filter_bits = ADI_4T_FILTER_BITS;
#endif
#if IIP
    const s16(*tbl_long_filt_list[4])[8] = { com_tbl_ipred_adi_long, com_tbl_ipred_adi_long + 32,com_tbl_ipred_adi_long + 64 , com_tbl_ipred_adi_long + 96 };
    const int long_filter_bits_list[4] = { 10, 10, 9, 10 };
    const int long_filter_offset_list[4] = { 512, 512 ,256, 512 };

#endif
    const int *mt = com_tbl_ipred_dxdy[ipm];

    const pel *src_ch = NULL;
    const s16 *filter;

    int offset_x[MAX_CU_SIZE], offset_y[MAX_CU_SIZE];
    int t_dx[MAX_CU_SIZE], t_dy[MAX_CU_SIZE];
    int i, j;
    int offset;
    int pos_max = w + h - 1;
    int p, pn, pn_n1, pn_p2;
#if IIP 
    int pn_n2, pn_n3, pn_p3, pn_p4;
#endif
#if EIPM
    if ((ipm < IPD_VER) || (ipm >= IPD_DIA_L_EXT && ipm <= IPD_VER_EXT))
#else
    if (ipm < IPD_VER)
#endif
    {
        src_ch = src_up;
        pos_max = w * 2 - 1;

        for (j = 0; j < h; j++) 
        {
            int dx;
            GET_REF_POS(mt[0], j + 1, dx, offset);
#if MIPF
            filter_idx = mipf_enable_flag ? (j < td ? is_small + 1 : is_small) : 3;
#if IIP
            filter = iip_flag ? (tbl_long_filt_list[filter_idx] + offset)[0] : (tbl_filt_list[filter_idx] + offset)[0];
#else
            filter = (tbl_filt_list[filter_idx] + offset)[0];
#endif
#else
            filter = (tbl_filt + offset)[0];
#endif
            for (i = 0; i < w; i++) 
            {
                int x = i + dx;
                pn_n1 = x - 1;
                p = x;
                pn = x + 1;
                pn_p2 = x + 2;

                pn_n1 = COM_MIN(pn_n1, pos_max);
                p = COM_MIN(p, pos_max);
                pn = COM_MIN(pn, pos_max);
                pn_p2 = COM_MIN(pn_p2, pos_max);
#if IIP 
                pn_n2 = x - 2;
                pn_n3 = x - 3;
                pn_p3 = x + 3;
                pn_p4 = x + 4;

                pn_n2 = COM_MIN(pn_n2, pos_max);
                pn_n3 = COM_MIN(pn_n3, pos_max);
                pn_p3 = COM_MIN(pn_p3, pos_max);
                pn_p4 = COM_MIN(pn_p4, pos_max);
#endif
#if MIPF
#if IIP 
                if (iip_flag)
                {
                    dst[i] = (pel)((src_ch[pn_n3] * filter[0] + src_ch[pn_n2] * filter[1] +
                        src_ch[pn_n1] * filter[2] + src_ch[p] * filter[3] +
                        src_ch[pn] * filter[4] + src_ch[pn_p2] * filter[5] +
                        src_ch[pn_p3] * filter[6] + src_ch[pn_p4] * filter[7] +
                        long_filter_offset_list[filter_idx]) >> long_filter_bits_list[filter_idx]);
                }
                else
                {
#endif
                    dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                        src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                        filter_offset_list[filter_idx]) >> filter_bits_list[filter_idx]);
#if IIP
                }
#endif
#else
                dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                    src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                    filter_offset) >> filter_bits);
#endif
            }
            dst += w;
        }
    } 
#if EIPM
    else if ((ipm > IPD_HOR && ipm < IPD_IPCM) || (ipm >= IPD_HOR_EXT && ipm < IPD_CNT))
#else
    else if (ipm > IPD_HOR)
#endif
    {
        src_ch = src_le;
        pos_max = h * 2 - 1;

        for (i = 0; i < w; i++) 
        {
            GET_REF_POS(mt[1], i + 1, t_dy[i], offset_y[i]);
        }

        for (j = 0; j < h; j++) 
        {
            for (i = 0; i < w; i++) 
            {
                int y = j + t_dy[i];
                pn_n1 = y - 1;
                p = y;
                pn = y + 1;
                pn_p2 = y + 2;

#if  MIPF
                filter_idx = mipf_enable_flag ? (i < td ? is_small + 1 : is_small) : 3;
#if IIP 
                filter = iip_flag ? (tbl_long_filt_list[filter_idx] + offset_y[i])[0] : (tbl_filt_list[filter_idx] + offset_y[i])[0];
#else
                filter = (tbl_filt_list[filter_idx] + offset_y[i])[0];
#endif
#else
                filter = (tbl_filt + offset_y[i])[0];
#endif

                pn_n1 = COM_MIN(pn_n1, pos_max);
                p = COM_MIN(p, pos_max);
                pn = COM_MIN(pn, pos_max);
                pn_p2 = COM_MIN(pn_p2, pos_max);
#if IIP 
                pn_n2 = y - 2;
                pn_n3 = y - 3;
                pn_p3 = y + 3;
                pn_p4 = y + 4;

                pn_n2 = COM_MIN(pn_n2, pos_max);
                pn_n3 = COM_MIN(pn_n3, pos_max);
                pn_p3 = COM_MIN(pn_p3, pos_max);
                pn_p4 = COM_MIN(pn_p4, pos_max);
#endif
#if MIPF
#if IIP 
                if (iip_flag)
                {
                    dst[i] = (pel)((src_ch[pn_n3] * filter[0] + src_ch[pn_n2] * filter[1] +
                        src_ch[pn_n1] * filter[2] + src_ch[p] * filter[3] +
                        src_ch[pn] * filter[4] + src_ch[pn_p2] * filter[5] +
                        src_ch[pn_p3] * filter[6] + src_ch[pn_p4] * filter[7] +
                        long_filter_offset_list[filter_idx]) >> long_filter_bits_list[filter_idx]);
                }
                else
                {
#endif
                    dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                        src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                        filter_offset_list[filter_idx]) >> filter_bits_list[filter_idx]);
#if IIP
                }
#endif
#else
                dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                    src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                    filter_offset) >> filter_bits);
#endif
            }
            dst += w;
        }
    } 
    else 
    {
        for (i = 0; i < w; i++) 
        {
            GET_REF_POS(mt[1], i + 1, t_dy[i], offset_y[i]);
            t_dy[i] = -t_dy[i];
        }
        for (j = 0; j < h; j++) 
        {
            GET_REF_POS(mt[0], j + 1, t_dx[j], offset_x[j]);
            t_dx[j] = -t_dx[j];
        }
#if MIPF
#if EIPM
        if (ipm < IPD_DIA_R || (ipm > IPD_VER_EXT && ipm <= IPD_DIA_R_EXT))
#else
        if (ipm < IPD_DIA_R)
#endif
        {
#endif
        for (j = 0; j < h; j++) 
        {
#if  MIPF
            filter_idx = mipf_enable_flag ? (j < td ? is_small + 1 : is_small) : 3;
#endif
            for (i = 0; i < w; i++) 
            {
                int x = i + t_dx[j];
                int y = j + t_dy[i];

                if (y <= -1) 
                {
                    src_ch = src_up;
                    offset = offset_x[j];
                    pos_max = w * 2 - 1;

                    pn_n1 = x + 1;
                    p = x;
                    pn = x - 1;
                    pn_p2 = x - 2;
#if IIP 
                    pn_n2 = x + 2;
                    pn_n3 = x + 3;
                    pn_p3 = x - 3;
                    pn_p4 = x - 4;

#endif
                } 
                else 
                {
                    src_ch = src_le;
                    offset = offset_y[i];
                    pos_max = h * 2 - 1;

                    pn_n1 = y + 1;
                    p = y;
                    pn = y - 1;
                    pn_p2 = y - 2;
#if IIP
                    pn_n2 = y + 2;
                    pn_n3 = y + 3;
                    pn_p3 = y - 3;
                    pn_p4 = y - 4;

#endif
                }

#if  MIPF
#if IIP
                filter = iip_flag ? (tbl_long_filt_list[filter_idx] + offset)[0] : (tbl_filt_list[filter_idx] + offset)[0];
#else
                filter = (tbl_filt_list[filter_idx] + offset)[0];
#endif
#else
                filter = (tbl_filt + offset)[0];
#endif

                pn_n1 = COM_MIN(pn_n1, pos_max);
                p = COM_MIN(p, pos_max);
                pn = COM_MIN(pn, pos_max);
                pn_p2 = COM_MIN(pn_p2, pos_max);
#if IIP
                pn_n2 = COM_MIN(pn_n2, pos_max);
                pn_n3 = COM_MIN(pn_n3, pos_max);
                pn_p3 = COM_MIN(pn_p3, pos_max);
                pn_p4 = COM_MIN(pn_p4, pos_max);
#endif
#if MIPF
#if IIP
                if (iip_flag)
                {
                    dst[i] = (pel)((src_ch[pn_n3] * filter[0] + src_ch[pn_n2] * filter[1] +
                        src_ch[pn_n1] * filter[2] + src_ch[p] * filter[3] +
                        src_ch[pn] * filter[4] + src_ch[pn_p2] * filter[5] +
                        src_ch[pn_p3] * filter[6] + src_ch[pn_p4] * filter[7] +
                        long_filter_offset_list[filter_idx]) >> long_filter_bits_list[filter_idx]);
                }
                else
                {
#endif
                    dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                        src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                        filter_offset_list[filter_idx]) >> filter_bits_list[filter_idx]);
#if IIP
                }
#endif
#else
                dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                    src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                    filter_offset) >> filter_bits);
#endif
            }
            dst += w;
        }
#if MIPF
        }
        else
        {
            for (j = 0; j < h; j++)
            {
                for (i = 0; i < w; i++)
                {
                    int x = i + t_dx[j];
                    int y = j + t_dy[i];

                    if (y <= -1)
                    {
                        src_ch = src_up;
                        offset = offset_x[j];
                        pos_max = w * 2 - 1;

                        pn_n1 = x + 1;
                        p = x;
                        pn = x - 1;
                        pn_p2 = x - 2;
#if IIP
                        pn_n2 = x + 2;
                        pn_n3 = x + 3;
                        pn_p3 = x - 3;
                        pn_p4 = x - 4;

#endif
                    }
                    else
                    {
                        src_ch = src_le;
                        offset = offset_y[i];
                        pos_max = h * 2 - 1;

                        pn_n1 = y + 1;
                        p = y;
                        pn = y - 1;
                        pn_p2 = y - 2;
#if IIP
                        pn_n2 = y + 2;
                        pn_n3 = y + 3;
                        pn_p3 = y - 3;
                        pn_p4 = y - 4;

#endif
                    }

#if  MIPF
                    filter_idx = mipf_enable_flag ? (i < td ? is_small + 1 : is_small) : 3;
#if IIP
                    filter = iip_flag ? (tbl_long_filt_list[filter_idx] + offset)[0] : (tbl_filt_list[filter_idx] + offset)[0];
#else
                    filter = (tbl_filt_list[filter_idx] + offset)[0];
#endif
#else
                    filter = (tbl_filt + offset)[0];
#endif

                    pn_n1 = COM_MIN(pn_n1, pos_max);
                    p = COM_MIN(p, pos_max);
                    pn = COM_MIN(pn, pos_max);
                    pn_p2 = COM_MIN(pn_p2, pos_max);
#if MIPF
#if IIP
                    if (iip_flag)
                    {
                        dst[i] = (pel)((src_ch[pn_n3] * filter[0] + src_ch[pn_n2] * filter[1] +
                            src_ch[pn_n1] * filter[2] + src_ch[p] * filter[3] +
                            src_ch[pn] * filter[4] + src_ch[pn_p2] * filter[5] +
                            src_ch[pn_p3] * filter[6] + src_ch[pn_p4] * filter[7] +
                            long_filter_offset_list[filter_idx]) >> long_filter_bits_list[filter_idx]);
                    }
                    else
                    {
#endif
                        dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                            src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                            filter_offset_list[filter_idx]) >> filter_bits_list[filter_idx]);
#if IIP
                    }
#endif
#else
                    dst[i] = (pel)((src_ch[pn_n1] * filter[0] + src_ch[p] * filter[1] +
                        src_ch[pn] * filter[2] + src_ch[pn_p2] * filter[3] +
                        filter_offset) >> filter_bits);
#endif
                }
                dst += w;
            }
        }
#endif
    }
}

static const s32 g_ipf_pred_param[5][10] =
{
    { 24,  6,  2,  0,  0,  0,  0,  0,  0,  0 }, //4x4, 24, 0.5
    { 44, 25, 14,  8,  4,  2,  1,  1,  0,  0 }, //8x8, 44-1.2
    { 40, 27, 19, 13,  9,  6,  4,  3,  2,  1 }, //16x16, 40-1.8
    { 36, 27, 21, 16, 12,  9,  7,  5,  4,  3 }, //32x32, 36-2.5
    { 52, 44, 37, 31, 26, 22, 18, 15, 13, 11 }, //64x64
};

void ipf_core(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h)
{
    com_assert((MIN_CU_SIZE <= w) && (MIN_CU_SIZE <= h));
    com_assert(ipm < IPD_CNT);
    assert(com_tbl_log2[w] >= 2);
    assert(com_tbl_log2[h] >= 2);

    s32 filter_idx_hor = (s32)com_tbl_log2[w] - 2; //Block Size
    s32 filter_idx_ver = (s32)com_tbl_log2[h] - 2; //Block Size
    const s32 filter_range = 10;
    s32 ver_filter_range = filter_range;
    s32 hor_filter_range = filter_range;

    // TODO: g_ipf_pred_param doesn't support 128
    if (filter_idx_hor > 4)
    {
        filter_idx_hor = 4;
        hor_filter_range = 0; // don't use IPF at horizontal direction
    }
    if (filter_idx_ver > 4)
    {
        filter_idx_ver = 4;
        ver_filter_range = 0; // don't use IPF at vertical direction
    }

    const s32 *filter_hori_param = g_ipf_pred_param[filter_idx_hor];
    const s32 *filter_vert_param = g_ipf_pred_param[filter_idx_ver];
    const s32 par_shift = 6; //normalization factor
    const s32 par_scale = 1 << par_shift;
    const s32 par_offset = 1 << (par_shift - 1);

#if EIPM
    if ((IPD_DIA_L <= ipm && ipm <= IPD_DIA_R) || (34 <= ipm && ipm <= 50))
#else
    if (IPD_DIA_L <= ipm && ipm <= IPD_DIA_R)
#endif
    {
        // vertical mode use left reference pixels, don't use top reference
        ver_filter_range = 0;
    }
#if EIPM
    if ((ipm > IPD_DIA_R && ipm < IPD_IPCM) || (ipm > IPD_DIA_R_EXT && ipm < IPD_CNT))
#else
    if (IPD_DIA_R < ipm)
#endif
    {
        // horizontal mode use top reference pixels, don't use left reference
        hor_filter_range = 0;
    }

    s32 p_ref_lenth = w + h;
    s32 *p_ref_vector = com_malloc(p_ref_lenth * sizeof(s32));
    com_mset(p_ref_vector, 0, (w + h) * sizeof(s32));
    s32 *p_ref_vector_h = p_ref_vector + h;
    for( s32 i = 0; i < w; ++i )
    {
        p_ref_vector_h[i] = src_up[i];
    }
    for( s32 i = 1; i <= h; ++i )
    {
        p_ref_vector_h[-i] = src_le[i - 1];
    }

    for (s32 row = 0; row < h; ++row)
    {
        s32 pos = row * w;
        s32 coeff_top = (row < ver_filter_range) ? filter_vert_param[row] : 0;
        for (s32 col = 0; col < w; col++, pos++)
        {
            s32 coeff_left = (col < hor_filter_range) ? filter_hori_param[col] : 0;
            s32 coeff_cur = par_scale - coeff_left - coeff_top;
            s32 sample_val = (coeff_left* p_ref_vector_h[-row - 1] + coeff_top * p_ref_vector_h[col] + coeff_cur * dst[pos] + par_offset) >> par_shift;
            dst[pos] = sample_val;
        }
    }

    // Release memory
    com_mfree(p_ref_vector);
}

void clip_pred(pel *dst, const int w, const int h, int bit_depth)
{
    com_assert(NULL != dst);
    for( int i = 0; i < h; i++ )
    {
        for( int j = 0; j < w; j++ )
        {
            dst[i * w + j] = COM_CLIP3(0, ((1 << bit_depth) - 1), dst[i * w + j]);
        }
    }
}

#if IIP
void iip_refine(pel *src_le, pel *src_up, pel *dst, int w, int h)
{
    int i, j;
    int coef_dst, coef_ref, dist;
    int bit_shift = 8;
    int filt_offset = 128;
    s32 *tle = malloc((h + 4) * sizeof(s32));
    s32 *tup = malloc((w + 4) * sizeof(s32));
    for (i = 0; i < h; i++)
    {
        tle[i + 2] = src_le[i];
    }
    for (i = 0; i < w; i++)
    {
        tup[i + 2] = src_up[i];
    }
    tle[0] = tle[1] = src_le[-1];
    tle[h + 2] = tle[h + 3] = src_le[h - 1];
    tup[0] = tup[1] = src_up[-1];
    tup[w + 2] = tup[w + 3] = src_up[w - 1];
    
    for (j = 0; j < h; j++)
    {
        for (i = 0; i < w; i++)
        {
            coef_dst = (i <= 1 || j <= 1) ? 216 : 204;
            coef_ref = (i <= 1 || j <= 1) ? 10 : 13;
            dist = (i == 0 || j == 0) ? 1 : 0;
            dst[i + j * w] = (s32)((tle[j + dist] + tle[j + 4 - dist] + tup[i + dist] + tup[i + 4 - dist]) * coef_ref + dst[i + j * w] * coef_dst + filt_offset) >> bit_shift;
        }
    }
    
    free(tle);
    free(tup);
}
#endif

void com_ipred(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h, int bit_depth, u16 avail_cu, u8 ipf_flag
#if MIPF
    , int mipf_enable_flag
#endif
#if IIP
    , u8 iip_flag
#endif
)
{
    assert(w <= 64 && h <= 64);

    switch(ipm)
    {
    case IPD_VER:
        ipred_vert(src_up, dst, w, h
#if IIP
            , iip_flag 
#endif
        );
        break;
    case IPD_HOR:
        ipred_hor(src_le, dst, w, h
#if IIP
            , iip_flag 
#endif
        );
        break;
    case IPD_DC:
        ipred_dc(src_le, src_up, dst, w, h, bit_depth, avail_cu
#if IIP
            , iip_flag 
#endif 
        );
        break;
    case IPD_PLN:
        ipred_plane(src_le, src_up, dst, w, h
#if IIP
            , iip_flag
#endif 
        );
        break;
    case IPD_BI:
        ipred_bi(src_le, src_up, dst, w, h
#if IIP
            , iip_flag 
#endif
        );
        break;
    default:
        ipred_ang(src_le, src_up, dst, w, h, ipm
#if MIPF
            , 1, mipf_enable_flag
#endif
#if IIP
            , iip_flag 
#endif
        );
        break;
    }
    if( ipf_flag )
    {
        assert((w < MAX_CU_SIZE) && (h < MAX_CU_SIZE));
        ipf_core(src_le, src_up, dst, ipm, w, h);
    }

#if IIP
    if (iip_flag )
    {
        assert((w * h >= MIN_IIP_BLK) && (w * h <= MAX_IIP_BLK));
        iip_refine(src_le, src_up, dst, w, h);
        
    }
#endif

    // Clip predicted value
#if UNIFIED_INTRA_CLIP
    int clipCondition = -1;
#if MIPF
    if (mipf_enable_flag)
    {
        clipCondition = ipf_flag
#if IIP
            || iip_flag
#endif
            || 1 // must clip in AVS3-Phase II
            ;
    }
    else
#endif
    {
        clipCondition = ipf_flag || (ipm == IPD_BI) || (ipm == IPD_PLN);
    }
    if(clipCondition)
#else
#if MIPF
#if IIP
    if (ipf_flag || ipm != IPD_DC || iip_flag)
#else
    if (ipf_flag || (ipm != IPD_VER && ipm != IPD_HOR  && ipm != IPD_DC))
#endif
#else
    if (ipf_flag || (ipm == IPD_BI) || (ipm == IPD_PLN))
#endif
#endif
    {
        clip_pred(dst, w, h, bit_depth);
    }
}

void com_ipred_uv(pel *src_le, pel *src_up, pel *dst, int ipm_c, int ipm, int w, int h, int bit_depth, u16 avail_cu
#if TSCPM
                  , int comp_id,  pel *reco_y, int stride_y, pel nb[N_C][N_REF][MAX_CU_SIZE * 3]
#endif
#if MIPF
                  , int mipf_enable_flag
#endif
#if PMC || EPMC
                  , pel *reco_u, int stride_u
#endif
#if IPF_CHROMA
                  , u8 ipfc_flag, CHANNEL_TYPE ch_type
#endif
)
{
    assert(w <= 64 && h <= 64);
#if CHROMA_NOT_SPLIT
    assert(w >= 4 && h >= 4);
#endif
#if TSCPM
    int is_above = IS_AVAIL(avail_cu, AVAIL_UP);
    int is_left = IS_AVAIL(avail_cu, AVAIL_LE);
#endif
    if(ipm_c == IPD_DM_C && COM_IPRED_CHK_CONV(ipm))
    {
        ipm_c = COM_IPRED_CONV_L2C(ipm);
    }
    switch(ipm_c)
    {
    case IPD_DM_C:
        switch(ipm)
        {
        case IPD_PLN:
            ipred_plane(src_le, src_up, dst, w, h
#if IIP
                , 0
#endif
            );
            break;
        default:
            ipred_ang(src_le, src_up, dst, w, h, ipm
#if MIPF
                , 0, mipf_enable_flag
#endif
#if IIP
                , 0
#endif
            );
            break;
        }
        // Clip
        clip_pred(dst, w, h, bit_depth);
        break;
    case IPD_DC_C:
        ipred_dc(src_le, src_up, dst, w, h, bit_depth, avail_cu
#if IIP
            , 0
#endif 
        );
        break;
    case IPD_HOR_C:
        ipred_hor(src_le, dst, w, h
#if IIP
            , 0
#endif
        );
        break;
    case IPD_VER_C:
        ipred_vert(src_up, dst, w, h
#if IIP
            , 0
#endif
        );
        break;
    case IPD_BI_C:
        ipred_bi(src_le, src_up, dst, w, h
#if IIP
            , 0
#endif
        );
        // Clip
        clip_pred(dst, w, h, bit_depth);
        break;
#if TSCPM || PMC || EPMC
    case IPD_TSCPM_C:
        ipred_tscpm(comp_id, dst, reco_y, stride_y, w,  h, is_above, is_left, bit_depth, nb
#if ENHANCE_TSPCM || PMC || EPMC
            , ipm_c
#endif
        );
        break;
#if ENHANCE_TSPCM
#if ENHANCE_LT_MODE
    case IPD_TSCPM_LT_C:
#endif
    case IPD_TSCPM_L_C:
    case IPD_TSCPM_T_C:
        ipred_tscpm(comp_id, dst, reco_y, stride_y, w, h, is_above, is_left, bit_depth, nb, ipm_c);
        break;
#endif
#if PMC
    case IPD_MCPM_C:
#if ENHANCE_LT_MODE
    case IPD_MCPM_LT_C:
#endif
    case IPD_MCPM_L_C:
    case IPD_MCPM_T_C:
        ipred_mcpm(comp_id, dst, reco_y, stride_y, w, h, is_above, is_left, bit_depth, nb, ipm_c,
#if EPMC
            0,
#endif
            reco_u, stride_u
        );
        break;
#endif
#if EPMC
    case IPD_EMCPM_C:
#if ENHANCE_LT_MODE
    case IPD_EMCPM_LT_C:
#endif
    case IPD_EMCPM_L_C:
    case IPD_EMCPM_T_C:
        ipred_mcpm(comp_id, dst, reco_y, stride_y, w, h, is_above, is_left, bit_depth, nb
            , ipm_c,
            MOD_IDX,
            reco_u, stride_u
        );
        break;
    case IPD_EMCPM2_C:
#if ENHANCE_LT_MODE
    case IPD_EMCPM2_LT_C:
#endif
    case IPD_EMCPM2_L_C:
    case IPD_EMCPM2_T_C:
        ipred_mcpm(comp_id, dst, reco_y, stride_y, w, h, is_above, is_left, bit_depth, nb
            , ipm_c,
            MOD2_IDX,
            reco_u, stride_u
        );
        break;
#endif
#endif
    default:
        printf("\n illegal chroma intra prediction mode\n");
        break;
    }

#if IPF_CHROMA
    if (ipfc_flag && ch_type == CHANNEL_LC)
    {
        switch(ipm_c)
        {
        case IPD_HOR_C:
        case IPD_TSCPM_L_C:
            ipf_core(src_le, src_up, dst, IPD_HOR, w, h);
            clip_pred(dst, w, h, bit_depth);
            break;
        case IPD_VER_C:
        case IPD_TSCPM_T_C:
            ipf_core(src_le, src_up, dst, IPD_VER, w, h);
            clip_pred(dst, w, h, bit_depth);
            break;
        default:
            break;
        }
    }
#endif
}

#if IPC
void get_ipc_linear(pel *dst, pel *src_up, pel *src_le, int *a, int *b, int *shift, int is_above, int is_left, int width, int height, int bit_depth, int lm_idx, int ch_type
#if IPC_SB8
    , int ipc_size
#endif
)
{
    pel *p_cur = NULL;
    pel *p_cur_left  = NULL;
    pel *p_cur_above = NULL;
    p_cur_left = src_le;
    p_cur_above = src_up;
#if IPC_SB8
    int blk_size = ((ch_type == Y_C) ? (IPC_POS_MAX_LUMA >> (ipc_size == 8)) : (IPC_POS_MAX_CHR >> (ipc_size == 8)));
#else
    int blk_size = ((ch_type==Y_C) ? IPC_POS_MAX_LUMA : IPC_POS_MAX_CHR);
#endif
    int subw = width > blk_size ? blk_size : width;
    int subh = height > blk_size ? blk_size : height;
    int min_dim = is_left && is_above ? min(subh, subw) : (is_left ? subh : subw);
    int num_steps = min_dim;
    int y_max = 0;
    int x_max = -MAX_INT;
    int y_min = 0;
    int x_min = MAX_INT;
    // four points start
    int ref_pixel_luma1[4]   = { -1, -1, -1, -1 };
    int ref_pixel_luma2[4] = { -1, -1, -1, -1 };
    switch (lm_idx)
    {
    case 1:
    if (is_above)
    {
        p_cur = p_cur_above;
        ref_pixel_luma1[0] = dst[1];
        ref_pixel_luma2[0] = p_cur[1];
        int idx = ((num_steps - 1) * subw) / min_dim;
        ref_pixel_luma1[1] = dst[COM_MIN(idx + 1, subw - 1)];
        ref_pixel_luma2[1] = p_cur[COM_MIN(idx + 1, subw - 1)];
        // using 4 points when only one border
        if (!is_left && width >= 4)
        {
            int step = subw >> 2;
            for (int i = 0; i < 4; i++)
            {
                ref_pixel_luma1[i] = dst[i * step];
                ref_pixel_luma2[i] = p_cur[i * step];
            }
        }
    }
    if (is_left)
    {
        p_cur = p_cur_left;
        ref_pixel_luma1[2] = dst[width];
        ref_pixel_luma2[2] = p_cur[1];
        int idx = ((num_steps - 1) * subh) / min_dim;
        ref_pixel_luma1[3] = dst[COM_MIN((idx + 1) * width, width * (subh - 1))];
        ref_pixel_luma2[3] = p_cur[COM_MIN(idx + 1, subh - 1)];
        // using 4 points when only one border
        if (!is_above && height >= 4)
        {
            int step = subh >> 2;
            for (int i = 0; i < 4; i++)
            {
                ref_pixel_luma1[i] = dst[i * step * width]; 
                ref_pixel_luma2[i] = p_cur[i * step];
            }
        }
    }
    break;
    case 2:
    if (is_above) 
    {
        p_cur = p_cur_above;
        // using 4 points when only one border
        if (width >= 4) 
        {
            int step = subw >> 2;
            for (int i = 0; i < 4; i++) 
            {
                ref_pixel_luma1[i] = dst[i * step];
                ref_pixel_luma2[i] = p_cur[i * step];
            }
        }
    }
    else
    {
        is_left = 0;
    }
    break;
    case 3:
    if (is_left) 
    {
        p_cur = p_cur_left;
        // using 4 points when only one border
        if (height >= 4) 
        {
            int step = subh >> 2;
            for (int i = 0; i < 4; i++) 
            {
                ref_pixel_luma1[i] = dst[i * step * width];
                ref_pixel_luma2[i] = p_cur[i * step];
            }
        }
    }
    else
    {
        is_above = 0;
    }
    break;   
    default:
        assert(0);
        break;
    }
    if (   (is_above &&  is_left)
        || (is_above && !is_left  && width  >= 4)
        || (is_left  && !is_above && height >= 4) )
    {
        int min_grp_idx[2] = { 0, 2 };
        int max_grp_idx[2] = { 1, 3 };
        int *tmp_min_grp = min_grp_idx;
        int *tmp_max_grp = max_grp_idx;
        if (ref_pixel_luma1[tmp_min_grp[0]] > ref_pixel_luma1[tmp_min_grp[1]]) SWAP_PIXEL(tmp_min_grp[0], tmp_min_grp[1], int);
        if (ref_pixel_luma1[tmp_max_grp[0]] > ref_pixel_luma1[tmp_max_grp[1]]) SWAP_PIXEL(tmp_max_grp[0], tmp_max_grp[1], int);
        if (ref_pixel_luma1[tmp_min_grp[0]] > ref_pixel_luma1[tmp_max_grp[1]]) SWAP_PIXEL(tmp_min_grp,    tmp_max_grp,    int *);
        if (ref_pixel_luma1[tmp_min_grp[1]] > ref_pixel_luma1[tmp_max_grp[0]]) SWAP_PIXEL(tmp_min_grp[1], tmp_max_grp[0], int);
        assert(ref_pixel_luma1[tmp_max_grp[0]] >= ref_pixel_luma1[tmp_min_grp[0]]);
        assert(ref_pixel_luma1[tmp_max_grp[0]] >= ref_pixel_luma1[tmp_min_grp[1]]);
        assert(ref_pixel_luma1[tmp_max_grp[1]] >= ref_pixel_luma1[tmp_min_grp[0]]);
        assert(ref_pixel_luma1[tmp_max_grp[1]] >= ref_pixel_luma1[tmp_min_grp[1]]);
        x_min = (ref_pixel_luma1  [tmp_min_grp[0]] + ref_pixel_luma1  [tmp_min_grp[1]] + 1 )>> 1;
        y_min = (ref_pixel_luma2[tmp_min_grp[0]] + ref_pixel_luma2[tmp_min_grp[1]] + 1) >> 1;
        x_max = (ref_pixel_luma1  [tmp_max_grp[0]] + ref_pixel_luma1  [tmp_max_grp[1]] + 1 )>> 1;
        y_max = (ref_pixel_luma2[tmp_max_grp[0]] + ref_pixel_luma2[tmp_max_grp[1]] + 1) >> 1;
    }
    else if (is_above)
    {
        for (int k = 0; k < 2; k++)
        {
            if (ref_pixel_luma1[k] > x_max)
            {
                x_max = ref_pixel_luma1[k];
                y_max = ref_pixel_luma2[k];
            }
            if (ref_pixel_luma1[k] < x_min)
            {
                x_min = ref_pixel_luma1[k];
                y_min = ref_pixel_luma2[k];
            }
        }
    }
    else if (is_left)
    {
        for (int k = 2; k < 4; k++)
        {
            if (ref_pixel_luma1[k] > x_max)
            {
                x_max = ref_pixel_luma1[k];
                y_max = ref_pixel_luma2[k];
            }
            if (ref_pixel_luma1[k] < x_min)
            {
                x_min = ref_pixel_luma1[k];
                y_min = ref_pixel_luma2[k];
            }
        }
    }
    // four points end
    if (is_left || is_above)
    {
        *a = 0;
        *shift = 16;
        int diff = x_max - x_min;
        int add = 0;
        int shift_div = 0;
        if (diff > 64)
        {
            shift_div = (bit_depth > 8) ? bit_depth - 6 : 2;
            add = shift_div ? 1 << (shift_div - 1) : 0;
            diff = (diff + add) >> shift_div;
            if (bit_depth == 10)
            {
                assert(shift_div == 4 && add == 8); // for default 10bit
            }
        }
        if (ch_type != Y_C)
        {
            if (abs(diff) <= 1<<(bit_depth-8) || abs(y_max - y_min) <= 1 << (bit_depth - 8))
            {
                *a = 0;
                *b = (y_max + y_min) / 2;
                return;
            }
        }
        if (diff > 0)
        {
            *a = ((y_max - y_min) * g_aiTscpmDivTable64[diff - 1] + add) >> shift_div;
        }
        *b = y_min - (((s64)(*a) * x_min) >> (*shift));
        if (ch_type != Y_C)
        {
            assert(*a != 0);
        }
    }
    if (!is_left && !is_above)
    {
        *a = 0;
        *b = 1 << (bit_depth - 1);
        *shift = 0;
        return;
    }
}
void com_inter_pred_correction(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h, int bit_depth, u16 avail_cu, int lm_idx ,int ch_type
#if IPC_SB8
    , int ipc_size
#endif
)
{
    int a, b, shift, i, j, temp_val;
    int is_above = IS_AVAIL(avail_cu, AVAIL_UP);
    int is_left = IS_AVAIL(avail_cu, AVAIL_LE);
    int max_val = (1 << bit_depth) - 1;
    get_ipc_linear(dst, src_up, src_le, &a, &b, &shift, is_above, is_left, w, h, bit_depth, lm_idx , ch_type
#if IPC_SB8
        , ipc_size
#endif
    );
    if (ch_type == Y_C)
    {
        for (j = 0; j < h; j++)
        {
            for (i = 0; i < w; i++)
            {
                temp_val = (((s64)a * dst[i]) >> (shift >= 0 ? shift : 0)) + b;
                dst[i] = COM_CLIP3(0, max_val, temp_val);
            }
            dst += w;
        }
    }
    else if (a == 0)
    {
        pel p= COM_CLIP3(0, max_val, b);
        com_mset_16b(dst, p, w*h);
    }
}
void pred_inter_pred_correction(COM_PIC *pic, u32 *map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE *mod_info_curr,
                       int x, int y, int cu_width, int cu_height, int bit_depth, int component, int pfIdx
#if IPC_SB8
    , int ipc_size
#endif
)
{
    u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, pic_width_in_scu, mod_info_curr->scup, map_scu);
    if ((component==0)||(component==1))
    {
        /* Y */
        int  s_rec = pic->stride_luma;
        pel *rec = pic->y + (y * s_rec) + x;
        com_get_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb, mod_info_curr->scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, Y_C);
        com_inter_pred_correction(nb[0][0] + STNUM, nb[0][1] + STNUM, mod_info_curr->pred[Y_C], mod_info_curr->ipm[PB0][0], cu_width, cu_height, bit_depth, avail_cu, pfIdx, Y_C
#if IPC_SB8
            , ipc_size
#endif
        );
    }
    if ((component==0)||(component==2))
    {
        int  s_rec = pic->stride_chroma;
        pel *rec;
        /* U */
        rec = pic->u + ((y >> 1) * s_rec) + (x >> 1);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, rec, s_rec, avail_cu, nb, mod_info_curr->scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, U_C);
        /* V */
        rec = pic->v + ((y >> 1) * s_rec) + (x >> 1);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, rec, s_rec, avail_cu, nb, mod_info_curr->scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, V_C);
        /*ic*/
        com_inter_pred_correction(nb[1][0] + STNUM, nb[1][1] + STNUM, mod_info_curr->pred[U_C], mod_info_curr->ipm[PB0][1], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu, pfIdx, U_C
#if IPC_SB8
            , ipc_size
#endif
        );
        com_inter_pred_correction(nb[2][0] + STNUM, nb[2][1] + STNUM, mod_info_curr->pred[V_C], mod_info_curr->ipm[PB0][1], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu, pfIdx, V_C 
#if IPC_SB8
            , ipc_size
#endif
        );
    }
}
#endif
#if INTERPF
void com_inter_filter(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h, int bit_depth, u16 avail_cu, int pfIdx)
{
    int coef_h = 0, coef_v = 0;
    int x, y;
    int wintra = 3;
    int winter = 5;
    int log2_w = com_tbl_log2[w];
    int log2_h = com_tbl_log2[h];

    pel *dst_start = dst;
    if (pfIdx == 2)
    {
        ipf_core(src_le, src_up, dst, IPD_BI, w, h);/*  component=Y/U_C/V_C  */
    }
    else
    {
        for (y = 0; y < h; y++)
        {
            for (x = 0; x < w; x++)
            {
                int predV = ((h - 1 - y)*src_up[x] + (y + 1)*src_le[h] + (h >> 1)) >> log2_h;
                int predH = ((w - 1 - x)*src_le[y] + (x + 1)*src_up[w] + (w >> 1)) >> log2_w;
                int predP1 = (predV + predH + 1) >> 1;

                dst[x] = (pel)((dst[x] * winter + predP1 * wintra + 4) >> 3);
            }
            dst += w;
        }
    }
    clip_pred(dst_start, w, h, bit_depth);
}

void pred_inter_filter(COM_PIC *pic, u32 *map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE *mod_info_curr,
                       int x, int y, int cu_width, int cu_height, int bit_depth, int component, int pfIdx)
{
    u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, pic_width_in_scu, mod_info_curr->scup, map_scu);

    if ((component==0)||(component==1))
    {
        /* Y */
        int  s_rec = pic->stride_luma;
        pel *rec = pic->y + (y * s_rec) + x;
        com_get_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb, mod_info_curr->scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, Y_C);
        com_inter_filter(nb[0][0] + STNUM, nb[0][1] + STNUM, mod_info_curr->pred[Y_C], mod_info_curr->ipm[PB0][0], cu_width, cu_height, bit_depth, avail_cu, pfIdx);
    }

    if ((component==0)||(component==2))
    {
        int  s_rec = pic->stride_chroma;
        pel *rec;
        /* U */
        rec = pic->u + ((y >> 1) * s_rec) + (x >> 1);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, rec, s_rec, avail_cu, nb, mod_info_curr->scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, U_C);
        /* V */
        rec = pic->v + ((y >> 1) * s_rec) + (x >> 1);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, rec, s_rec, avail_cu, nb, mod_info_curr->scup, map_scu, pic_width_in_scu, pic_height_in_scu, bit_depth, V_C);
        com_inter_filter(nb[1][0] + STNUM, nb[1][1] + STNUM, mod_info_curr->pred[U_C], mod_info_curr->ipm[PB0][1], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu, pfIdx);
        com_inter_filter(nb[2][0] + STNUM, nb[2][1] + STNUM, mod_info_curr->pred[V_C], mod_info_curr->ipm[PB0][1], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu, pfIdx);
    }
}

#endif

void com_get_mpm(int x_scu, int y_scu, u32 *map_scu, s8 *map_ipm, int scup, int pic_width_in_scu, u8 mpm[2])
{
    u8 ipm_l = IPD_DC, ipm_u = IPD_DC;
    int valid_l = 0, valid_u = 0;

    if(x_scu > 0 && MCU_GET_INTRA_FLAG(map_scu[scup - 1]) && MCU_GET_CODED_FLAG(map_scu[scup - 1]))
    {
        ipm_l = map_ipm[scup - 1];
        valid_l = 1;
    }

    if(y_scu > 0 && MCU_GET_INTRA_FLAG(map_scu[scup - pic_width_in_scu]) && MCU_GET_CODED_FLAG(map_scu[scup - pic_width_in_scu]))
    {
        ipm_u = map_ipm[scup - pic_width_in_scu];
        valid_u = 1;
    }
    mpm[0] = COM_MIN(ipm_l, ipm_u);
    mpm[1] = COM_MAX(ipm_l, ipm_u);
    if(mpm[0] == mpm[1])
    {
        mpm[0] = IPD_DC;
        mpm[1] = (mpm[1] == IPD_DC) ? IPD_BI : mpm[1];
    }
}

#if SAWP
#if SAWP_MPM_SIMP
void com_get_sawp_mpm(int x_scu, int y_scu, int cu_width_in_scu, int cu_height_in_scu, u32* map_scu, s8* map_ipm, int scup, int pic_width_in_scu, u8 sawp_mpm[SAWP_MPM_NUM], u8 awp_idx)
{
    int neb_addr[2];
    int neb_ipm[2] = { IPD_CNT, IPD_CNT };

    //! A: left neighbor
    neb_addr[0] = scup - 1;
    if (x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[0]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[0]]))
    {
        neb_ipm[0] = map_ipm[neb_addr[0]];
    }

    //! B: above neighbor
    neb_addr[1] = scup - pic_width_in_scu;
    if (y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[1]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[1]]))
    {
        neb_ipm[1] = map_ipm[neb_addr[1]];
    }

    for (int i = 0; i < 2; i++)
    {
        if (neb_ipm[i] != IPD_CNT)
        {
            if (neb_ipm[i] < 5)
            {
                neb_ipm[i] = IPD_CNT;
            }
            else if (neb_ipm[i] >= 31)
            {
                if (neb_ipm[i] <= 34)
                {
                    neb_ipm[i] = IPD_CNT;
                }
                else if (neb_ipm[i] < 44)
                {
                    neb_ipm[i] -= 30;
                }
                else if (neb_ipm[i] < 58)
                {
                    neb_ipm[i] -= 33;
                }
                else if (neb_ipm[i] < 65)
                {
                    neb_ipm[i] -= 34;
                }
                else
                {
                    neb_ipm[i] = IPD_CNT;
                }
            }
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (neb_ipm[i] == IPD_CNT)
        {
            neb_ipm[i] = com_tbl_awp_angle_mode[0][awp_idx % 8];
        }
    }

    if (neb_ipm[0] == neb_ipm[1])
    {
        if (neb_ipm[0] != com_tbl_awp_angle_mode[0][awp_idx % 8])
        {
            neb_ipm[0] = com_tbl_awp_angle_mode[0][awp_idx % 8];
        }
        else 
        {
            neb_ipm[1] = com_tbl_awp_angle_mode[1][awp_idx % 8];
        }
    }

    sawp_mpm[0] = COM_MIN(neb_ipm[0], neb_ipm[1]);
    sawp_mpm[1] = COM_MAX(neb_ipm[0], neb_ipm[1]);
}
#else // SAWP_MPM_SIMP

void com_get_sawp_mpm(int x_scu, int y_scu, int cu_width_in_scu, int cu_height_in_scu, u32* map_scu, s8* map_ipm, int scup, int pic_width_in_scu, u8 sawp_mpm[SAWP_MPM_NUM], u8 awp_idx)
{
    int neb_addr[8];
    int neb_ipm[10] = { IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT, IPD_CNT };

    //! F: left-below neighbor (inside)
    neb_addr[0] = scup + (cu_height_in_scu - 1) * pic_width_in_scu - 1;
    if (x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[0]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[0]]))
    {
        neb_ipm[0] = map_ipm[neb_addr[0]];
    }

    //! G: above-right neighbor (inside)
    neb_addr[1] = scup - pic_width_in_scu + cu_width_in_scu - 1;
    if (y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[1]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[1]]))
    {
        neb_ipm[1] = map_ipm[neb_addr[1]];
    }

    //! C: above-right neighbor (outside)
    neb_addr[2] = scup - pic_width_in_scu + cu_width_in_scu;
    if (y_scu > 0 && x_scu + cu_width_in_scu < pic_width_in_scu && MCU_GET_CODED_FLAG(map_scu[neb_addr[2]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[2]]))
    {
        neb_ipm[2] = map_ipm[neb_addr[2]];
    }

    //! A: left neighbor
    neb_addr[3] = scup - 1;
    if (x_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[3]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[3]]))
    {
        neb_ipm[3] = map_ipm[neb_addr[3]];
    }

    //! B: above neighbor
    neb_addr[4] = scup - pic_width_in_scu;
    if (y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[4]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[4]]))
    {
        neb_ipm[4] = map_ipm[neb_addr[4]];
    }

    //! D: above-left neighbor
    neb_addr[5] = scup - pic_width_in_scu - 1;
    if (x_scu > 0 && y_scu > 0 && MCU_GET_CODED_FLAG(map_scu[neb_addr[5]]) && MCU_GET_INTRA_FLAG(map_scu[neb_addr[5]]))
    {
        neb_ipm[5] = map_ipm[neb_addr[5]];
    }

    neb_ipm[6] = com_tbl_awp_angle_mode[0][awp_idx % 8];
    neb_ipm[7] = com_tbl_awp_angle_mode[1][awp_idx % 8];
    neb_ipm[8] = com_tbl_awp_angle_mode[2][awp_idx % 8];
    neb_ipm[9] = com_tbl_awp_angle_mode[3][awp_idx % 8];

    for (int i = 0; i < 10; i++)
    {
        if (neb_ipm[i] != IPD_CNT)
        {
            if (neb_ipm[i] < 4)
            {
                neb_ipm[i] = IPD_CNT;
            }
            else if (neb_ipm[i] >= 32)
            {
                if (neb_ipm[i] <= IPD_IPCM)
                {
                    neb_ipm[i] = IPD_CNT;
                }
                else if (neb_ipm[i] < 44)
                {
                    neb_ipm[i] -= 30;
                }
                else if (neb_ipm[i] < 58)
                {
                    neb_ipm[i] -= 33;
                }
                else
                {
                    neb_ipm[i] -= 34;
                }
            }
        }
    }

    int temp_mpm[4];
    int mpm_num = 0;
    for (int i = 0; i < 10; i++)
    {
        if (neb_ipm[i] != IPD_CNT)
        {
            BOOL bsame = FALSE;
            for (int j = 0; j < mpm_num; j++)
            {
                if (temp_mpm[j] == neb_ipm[i])
                {
                    bsame = TRUE;
                    break;
                }
            }
            if (!bsame)
            {
                temp_mpm[mpm_num] = neb_ipm[i];
                mpm_num++;
            }
        }
        if (mpm_num == SAWP_MPM_NUM)
        {
            break;
        }
    }
    assert(mpm_num == SAWP_MPM_NUM);
    for (int i = 0; i < SAWP_MPM_NUM; i++)
    {
        assert(temp_mpm[i] >= 4 && temp_mpm[i] <= 31);
    }
    
    for (int i = 0; i < SAWP_MPM_NUM; i++)
    {
        int smallest = IPD_CNT;
        int smallest_pos = -1;
        for (int j = 0; j < SAWP_MPM_NUM; j++)
        {
            if (temp_mpm[j] < smallest)
            {
                smallest = temp_mpm[j];
                smallest_pos = j;
            }
        }
        sawp_mpm[i] = smallest;
        temp_mpm[smallest_pos] = IPD_CNT;
    }
}
#endif // SAWP_MPM_SIMP
#endif // SAWP

#if FIMC
void com_get_cntmpm(int x_scu, int y_scu, u32* map_scu, s8* map_ipm, int scup, int pic_width_in_scu, u8 mpm[2], COM_CNTMPM * cntmpm)
{
    mpm[0] = cntmpm->modeT[0];
    mpm[1] = cntmpm->modeT[1];

    // keep mpm[0] < mpm[1]
    if (mpm[0] > mpm[1])
    {
        u8 tMode = mpm[0];
        mpm[0] = mpm[1];
        mpm[1] = tMode;
    }
    assert(mpm[0] < mpm[1]);
}
#endif
