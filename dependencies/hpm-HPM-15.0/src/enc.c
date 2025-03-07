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

#include <math.h>

#include "enc_def.h"
#include "enc_eco.h"
#include "com_df.h"
#include "enc_mode.h"
#include "com_util.h"
#include "com_sao.h"
#include "enc_EncAdaptiveLoopFilter.h"
#if USE_IBC
#include "enc_ibc_hash_wrapper.h"
#endif
#if ESAO || CCSAO
#include "com_esao.h"
#include "enc_esao.h"
#endif 
#if USE_SP    
extern  void release_sp_instance(void* sp_encoder);
extern  void sp_lcu_hash_save(void* sp_encoder, int x_lcu_start, int y_lcu_start);
#endif
#if ALO
#include "enc_optimization_wrapper.h"
#endif

/* Convert ENC into ENC_CTX */
#define ENC_ID_TO_CTX_R(id, ctx) \
    com_assert_r((id)); \
    (ctx) = (ENC_CTX *)id; \
    com_assert_r((ctx)->magic == ENC_MAGIC_CODE);

/* Convert ENC into ENC_CTX with return value if assert on */
#define ENC_ID_TO_CTX_RV(id, ctx, ret) \
    com_assert_rv((id), (ret)); \
    (ctx) = (ENC_CTX *)id; \
    com_assert_rv((ctx)->magic == ENC_MAGIC_CODE, (ret));


#if RPL_GOP32
static const s8 tbl_poc_gop_offset[9][31] =
{
    {  -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    {  -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    {  -4,   -6,   -7,   -5,   -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    {  -8,   -12, -14,  -15,  -13,  -10,  -11,   -9,   -4,   -6,   -7,   -5,   -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 16 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 20 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 24 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 28 */
    { -16,  -24,  -28,  -30,  -31,  -29,  -26,  -27,  -25,  -20,  -22,  -23,  -21,  -18,  -19,  -17,   -8,  -12,  -14,  -15,  -13,  -10,  -11,   -9,   -4,   -6,   -7,   -5,   -2,   -3,   -1}   /* gop_size = 32 */
};
#else
static const s8 tbl_poc_gop_offset[5][15] =
{
    { -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 2 */
    { -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 4 */
    { -4,   -6,   -7,   -5,   -2,   -3,   -1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 8 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    { -8,   -12, -14,  -15,  -13,  -10,  -11,   -9,   -4,   -6,   -7,   -5,   -2,   -3,   -1}   /* gop_size = 16 */
};
#endif

static const s8 tbl_ref_depth[5][15] =
{
    /* gop_size = 2 */
    { FRM_DEPTH_1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 4 */
    { FRM_DEPTH_1, FRM_DEPTH_2, FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 8 */
    {
        FRM_DEPTH_1, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* gop_size = 12 */
    /* gop_size = 16 */
    {
        FRM_DEPTH_1, FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, \
        FRM_DEPTH_4,  FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4
    }
};
static const s8 tbl_slice_ref[5][15] =
{
    /* gop_size = 2 */
    { 1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 4 */
    { 1, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 8 */
    { 1, 1, 0, 0, 1, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 16 */
    {1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0}
};

static const s8 tbl_slice_depth_P[GOP_P] =
{ FRM_DEPTH_3,  FRM_DEPTH_2, FRM_DEPTH_3,  FRM_DEPTH_2, FRM_DEPTH_3,  FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_1 };

#if RPL_GOP32
static const s8 tbl_slice_depth[9][31] =
{
    /* gop_size = 2 */
    {
        FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 4 */
    {
        FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 8 */
    {
        FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4,\
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 12 */
    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 16 */
    {
        FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, \
        FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 20*/
    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 24 */
    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 28 */
    {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 32 */
    {
        FRM_DEPTH_2, 
        FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, \
                     FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, \
        FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, \
                     FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6, FRM_DEPTH_5, FRM_DEPTH_6, FRM_DEPTH_6
    },
};
#else
static const s8 tbl_slice_depth[5][15] =
{
    /* gop_size = 2 */
    {
        FRM_DEPTH_2, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 4 */
    {
        FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_3, 0xFF, 0xFF, 0xFF, 0xFF, \
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 8 */
    {
        FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_4,\
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    },
    /* gop_size = 12 */
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    /* gop_size = 16 */
    {
        FRM_DEPTH_2, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, \
        FRM_DEPTH_5, FRM_DEPTH_3, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5, FRM_DEPTH_4, FRM_DEPTH_5, FRM_DEPTH_5
    }
};
#endif


static ENC_CTX * ctx_alloc(void)
{
    ENC_CTX * ctx;
    ctx = (ENC_CTX*)com_malloc_fast(sizeof(ENC_CTX));
    com_assert_rv(ctx, NULL);
    com_mset_x64a(ctx, 0, sizeof(ENC_CTX));
    return ctx;
}

static void ctx_free(ENC_CTX * ctx)
{
    com_mfree_fast(ctx);
}

static ENC_CORE * core_alloc(void)
{
    ENC_CORE * core;
    int i, j;
    core = (ENC_CORE *)com_malloc_fast(sizeof(ENC_CORE));
    com_assert_rv(core, NULL);
    com_mset_x64a(core, 0, sizeof(ENC_CORE));
    for(i = 0; i < MAX_CU_DEPTH; i++)
    {
        for(j = 0; j < MAX_CU_DEPTH; j++)
        {
            enc_create_cu_data(&core->cu_data_best[i][j], i, j
#if USE_SP
                , 1,1
#endif
            );
            enc_create_cu_data(&core->cu_data_temp[i][j], i, j
#if USE_SP
                , 1,1
#endif
            );
        }
    }
#if INTER_ME_MVLIB
    for (i = 0; i < MAX_NUM_MVR; i++)
    {
        core->s_enc_me_mvlib[i].uni_mv_list_max_size = 64;
        core->s_enc_me_mvlib[i].uni_mv_list = (BLK_UNI_MV_INFO*)com_malloc_fast(core->s_enc_me_mvlib[i].uni_mv_list_max_size * sizeof(BLK_UNI_MV_INFO));
        core->s_enc_me_mvlib[i].uni_mv_list_idx = 0;
        core->s_enc_me_mvlib[i].uni_mv_list_size = 0;
    }
#endif
#if ENC_ME_IMP
    core->affMVListMaxSize = 8;
    core->affMVList = (AffineMvInfo*)com_malloc_fast(core->affMVListMaxSize * sizeof(AffineMvInfo));
    core->affMVListIdx = 0;
    core->affMVListSize = 0;
#endif
    return core;
}

static void core_free(ENC_CORE * core)
{
    int i, j;
    for(i = 0; i < MAX_CU_DEPTH; i++)
    {
        for(j = 0; j < MAX_CU_DEPTH; j++)
        {
            enc_delete_cu_data(&core->cu_data_best[i][j]
#if USE_SP
                , 1,1
#endif
            );
            enc_delete_cu_data(&core->cu_data_temp[i][j]
#if USE_SP
                , 1,1
#endif
            );
        }
    }
#if INTER_ME_MVLIB
    for (i = 0; i < MAX_NUM_MVR; i++)
    {
        com_mfree_fast(core->s_enc_me_mvlib[i].uni_mv_list);
    }
#endif
#if ENC_ME_IMP
    com_mfree_fast(core->affMVList);
#endif
    com_mfree_fast(core);
}

static int set_init_param(ENC_PARAM * param, ENC_PARAM * param_input)
{
    com_mcpy(param, param_input, sizeof(ENC_PARAM));
    /* check input parameters */
    com_assert_rv(param->pic_width > 0 && param->pic_height > 0, COM_ERR_INVALID_ARGUMENT);
    com_assert_rv((param->pic_width & (MIN_CU_SIZE-1)) == 0,COM_ERR_INVALID_ARGUMENT);
    com_assert_rv((param->pic_height & (MIN_CU_SIZE-1)) == 0,COM_ERR_INVALID_ARGUMENT);
    com_assert_rv(param->qp >= (MIN_QUANT - (8 * (param->bit_depth_internal - 8))), COM_ERR_INVALID_ARGUMENT);
    com_assert_rv(param->qp <= MAX_QUANT_BASE, COM_ERR_INVALID_ARGUMENT); // this assertion is align with the constraint for input QP
    com_assert_rv(param->i_period >= 0,COM_ERR_INVALID_ARGUMENT);
    if( !param->disable_hgop )
    {
#if RPL_GOP32
        int b_frames_available = param->max_b_frames == 0  || param->max_b_frames == 1
                              || param->max_b_frames == 3  || param->max_b_frames == 7 
                              || param->max_b_frames == 15 || param->max_b_frames == 31;
        com_assert_rv(b_frames_available, COM_ERR_INVALID_ARGUMENT);
#else
        com_assert_rv(param->max_b_frames == 0 || param->max_b_frames == 1 || \
                      param->max_b_frames == 3 || param->max_b_frames == 7 || \
                      param->max_b_frames == 15, COM_ERR_INVALID_ARGUMENT);
#endif
        if(param->max_b_frames != 0)
        {
            if(param->i_period % (param->max_b_frames + 1) != 0)
            {
                com_assert_rv(0, COM_ERR_INVALID_ARGUMENT);
            }
        }
    }
    /* set default encoding parameter */
    param->f_ifrm         = 0;
    param->use_pic_sign   = 0;
    param->gop_size       = param->max_b_frames +1;
    return COM_OK;
}

static void set_cnkh(ENC_CTX * ctx, COM_CNKH * cnkh, int ver, int ctype)
{
    cnkh->ver = ver;
    cnkh->ctype = ctype;
    cnkh->broken = 0;
}

int load_wq_matrix(char *fn, u8 *m4x4_out, u8 *m8x8_out)
{
    int i, m4x4[16], m8x8[64];
    sscanf(fn, "[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]\n",
           &m4x4[ 0], &m4x4[ 1], &m4x4[ 2], &m4x4[ 3], &m4x4[ 4], &m4x4[ 5], &m4x4[ 6], &m4x4[ 7],
           &m4x4[ 8], &m4x4[ 9], &m4x4[10], &m4x4[11], &m4x4[12], &m4x4[13], &m4x4[14], &m4x4[15],
           &m8x8[ 0], &m8x8[ 1], &m8x8[ 2], &m8x8[ 3], &m8x8[ 4], &m8x8[ 5], &m8x8[ 6], &m8x8[ 7],
           &m8x8[ 8], &m8x8[ 9], &m8x8[10], &m8x8[11], &m8x8[12], &m8x8[13], &m8x8[14], &m8x8[15],
           &m8x8[16], &m8x8[17], &m8x8[18], &m8x8[19], &m8x8[20], &m8x8[21], &m8x8[22], &m8x8[23],
           &m8x8[24], &m8x8[25], &m8x8[26], &m8x8[27], &m8x8[28], &m8x8[29], &m8x8[30], &m8x8[31],
           &m8x8[32], &m8x8[33], &m8x8[34], &m8x8[35], &m8x8[36], &m8x8[37], &m8x8[38], &m8x8[39],
           &m8x8[40], &m8x8[41], &m8x8[42], &m8x8[43], &m8x8[44], &m8x8[45], &m8x8[46], &m8x8[47],
           &m8x8[48], &m8x8[49], &m8x8[50], &m8x8[51], &m8x8[52], &m8x8[53], &m8x8[54], &m8x8[55],
           &m8x8[56], &m8x8[57], &m8x8[58], &m8x8[59], &m8x8[60], &m8x8[61], &m8x8[62], &m8x8[63]);

    for (i = 0; i < 16; i++)
    {
        m4x4_out[i] = m4x4[i];
    }
    for (i = 0; i < 64; i++)
    {
        m8x8_out[i] = m8x8[i];
    }

    return COM_OK;
}

static int find_level_id_idx( int level_id )
{
    static u8 level_id_tbl[42] = { 0x00, 0x10, 0x12, 0x14, 0x20,
                                   0x22, 0x40, 0x42, 0x41, 0x43,
                                   0x44, 0x46, 0x45, 0x47, 0x48,
                                   0x4a, 0x49, 0x4b, 0x50, 0x52,
                                   0x51, 0x53, 0x54, 0x56, 0x55,
                                   0x57, 0x58, 0x5a, 0x59, 0x5b,
                                   0x60, 0x62, 0x61, 0x63, 0x64,
                                   0x66, 0x65, 0x67, 0x68, 0x6a,
                                   0x69, 0x6b };

    int idx = -1;
    int i;
    for (i = 0; i < 42; i++)
    {
        if (level_id_tbl[i] == level_id)
            idx = i;
    }
    assert(idx > 0 && idx < 42);
    return idx;
}

static void set_sqh(ENC_CTX * ctx, COM_SQH * sqh)
{
    int i, level_id_idx;
    static int fps_code_tbl[14] = {-1, -1, 24, 25, -1, 30, 50, -1, 60, 100, 120, 200, 240, 300 };
    static unsigned int max_bbv_size_tbl[42] = {         0,   1507328,   2015232,   2506752,   6012928, 
                                                  10010624,  12009472,  30015488,  12009472,  30015488,
                                                  20004864,  50003968,  20004864,  50003968,  25001984,
                                                 100007936,  25001984, 100007936,  25001984, 100007936,
                                                  25001984, 100007936,  40009728, 160006144,  40009728,
                                                 160006144,  60014592, 240009216,  60014592, 240009216,
                                                  60014592, 240009216,  60014592,	240009216, 120012800,
                                                 480002048, 120012800, 480002048, 240009216, 800014336,
                                                 240009216,	800014336        };
    static unsigned int max_bitrate_tbl[42] = {          0,   1500000,   2000000,   2500000,   6000000,
                                                  10000000,  12000000,  30000000,  12000000,  30000000,
                                                  20000000,  50000000,  20000000,  50000000,  25000000,  
                                                 100000000,  25000000, 100000000,  25000000, 100000000,
                                                  25000000, 100000000,  40000000, 160000000,  40000000, 
                                                 160000000,  60000000, 240000000,  60000000, 240000000,
                                                  60000000, 240000000,  60000000,	240000000, 120000000,
                                                 480000000, 120000000, 480000000, 240000000, 800000000,
                                                 240000000,	800000000       };
    static unsigned int max_dpb_size_in_bits_tbl[42] = {          
                                                         0,   MAX_INT,   MAX_INT,   MAX_INT,   MAX_INT,
                                                   MAX_INT,  14622720,  14622720,  16711680,  16711680,
                                                  14622720,  14622720,  16711680,  16711680,  14622720,  
                                                  14622720,  16711680,  16711680,  58490880,  58490880,
                                                  66846720,  66846720,  58490880,  58490880,  66846720,
                                                  66846720,  58490880,  58490880,  66846720,  66846720,
                                                 226492416, 226492416, 301989888, 301989888, 226492416,
                                                 226492416, 301989888, 301989888, 226492416, 226492416,
                                                 301989888, 301989888        };

#if PHASE_2_PROFILE
    sqh->profile_id = ctx->param.profile;
#else
    sqh->profile_id = PROFILE_ID;
#endif

#if CFG_LEVEL_ID
    sqh->level_id = ctx->param.level_id;
#else
    sqh->level_id = LEVEL_ID;
#endif
    level_id_idx = find_level_id_idx( sqh->level_id );
    sqh->progressive_sequence = 1;
    sqh->field_coded_sequence = 0;
#if LIBVC_ON
    sqh->library_stream_flag = ctx->rpm.libvc_data->is_libpic_processing ? 1 : 0;
    sqh->library_picture_enable_flag = (u8)ctx->param.library_picture_enable_flag;
    sqh->duplicate_sequence_header_flag = 1;
#endif
    sqh->sample_precision = ctx->param.bit_depth_input == 8 ? 1 : 2;
    sqh->aspect_ratio = 1;

    sqh->frame_rate_code = 15;

    for (i = 0; i < 14; i++)
    {
        if (ctx->param.fps == fps_code_tbl[i])
        {
            sqh->frame_rate_code = (u8)i;
            break;
        }
    }

    sqh->bit_rate_upper = (max_bitrate_tbl[level_id_idx] / 400) >> 18;
    sqh->bit_rate_lower = (max_bitrate_tbl[level_id_idx] / 400) - (sqh->bit_rate_upper << 18);
    sqh->low_delay = ctx->param.max_b_frames == 0 ? 1 : 0;
    sqh->temporal_id_enable_flag = 1;
    sqh->bbv_buffer_size = max_bbv_size_tbl[level_id_idx] / (1024 * 16);
    sqh->max_dpb_size = ctx->rpm.max_pb_size = min(16, max_dpb_size_in_bits_tbl[level_id_idx] / (ctx->param.pic_width * ctx->param.pic_height));

#if HLS_RPL
    sqh->rpls_l0_num = ctx->param.rpls_l0_cfg_num;
    sqh->rpls_l1_num = ctx->param.rpls_l1_cfg_num;
    sqh->rpl1_index_exist_flag = 1;
    sqh->rpl1_same_as_rpl0_flag = 0;

    memcpy(sqh->rpls_l0, ctx->param.rpls_l0, ctx->param.rpls_l0_cfg_num * sizeof(sqh->rpls_l0[0]));
    memcpy(sqh->rpls_l1, ctx->param.rpls_l1, ctx->param.rpls_l1_cfg_num * sizeof(sqh->rpls_l1[0]));
#endif

    sqh->horizontal_size = ctx->param.horizontal_size;
    sqh->vertical_size   = ctx->param.vertical_size;
    sqh->log2_max_cu_width_height = (u8)ctx->info.log2_max_cuwh;
    sqh->min_cu_size              = ctx->param.min_cu_size;
    sqh->max_part_ratio           = ctx->param.max_part_ratio;
    sqh->max_split_times          = ctx->param.max_split_times;
    sqh->min_qt_size              = ctx->param.min_qt_size;
    sqh->max_bt_size              = ctx->param.max_bt_size;
    sqh->max_eqt_size             = ctx->param.max_eqt_size;
    sqh->max_dt_size              = ctx->param.max_dt_size;
#if IPCM
    sqh->ipcm_enable_flag = ctx->param.ipcm_enable_flag;
#endif
    sqh->amvr_enable_flag = (u8)ctx->param.amvr_enable_flag;
    sqh->ipf_enable_flag = ctx->param.ipf_flag;
    sqh->affine_enable_flag = (u8)ctx->param.affine_enable_flag;
#if SMVD
    sqh->smvd_enable_flag = (u8)ctx->param.smvd_enable_flag;
#endif
    sqh->num_of_hmvp_cand = (u8)ctx->param.num_of_hmvp_cand;
    sqh->umve_enable_flag = (u8)ctx->param.umve_enable_flag;
#if EXT_AMVR_HMVP
    sqh->emvr_enable_flag = (u8)ctx->param.emvr_enable_flag;
#endif
#if TSCPM
    sqh->tscpm_enable_flag = (u8)ctx->param.tscpm_enable_flag;
#endif
#if DT_PARTITION
    sqh->dt_intra_enable_flag = (u8)ctx->param.dt_intra_enable_flag;
#endif
    sqh->position_based_transform_enable_flag = (u8)ctx->param.position_based_transform_enable_flag;
    sqh->sample_adaptive_offset_enable_flag = (u8)ctx->param.sample_adaptive_offset_enable_flag;
    sqh->wq_enable = ctx->param.wq_enable;
    sqh->seq_wq_mode = ctx->param.seq_wq_mode;

    if (sqh->wq_enable)
    {
        if (sqh->seq_wq_mode)
        {
            load_wq_matrix(ctx->param.seq_wq_user, sqh->wq_4x4_matrix, sqh->wq_8x8_matrix);
        }
        else
        {
            memcpy(sqh->wq_4x4_matrix, tab_WqMDefault4x4, sizeof(tab_WqMDefault4x4));
            memcpy(sqh->wq_8x8_matrix, tab_WqMDefault8x8, sizeof(tab_WqMDefault8x8));
        }
    }

    sqh->secondary_transform_enable_flag = (u8)ctx->param.secondary_transform_enable_flag;
    sqh->chroma_format = ctx->param.chroma_format;
    sqh->encoding_precision = ctx->param.encoding_precision;

    sqh->adaptive_leveling_filter_enable_flag = (u8)ctx->param.adaptive_leveling_filter_enable_flag;

#if PHASE_2_PROFILE
    if (sqh->profile_id == 0x30 || sqh->profile_id == 0x32)
    {
        // intra
#if EIPM
        sqh->eipm_enable_flag = (u8)ctx->param.eipm_enable_flag;
#endif
#if PMC
        sqh->pmc_enable_flag = (u8)ctx->param.pmc_enable_flag;
#endif
#if ENHANCE_TSPCM
        sqh->enhance_tscpm_enable_flag = (u8)ctx->param.enhance_tscpm_enable_flag;
#endif
#if IIP
        sqh->iip_enable_flag = (u8)ctx->param.iip_enable_flag;
#endif
#if SAWP
        sqh->sawp_enable_flag = (u8)ctx->param.sawp_enable_flag;
#endif
#if MIPF
        sqh->mipf_enable_flag = (u8)ctx->param.mipf_flag;
#endif
#if IPF_CHROMA
        sqh->chroma_ipf_enable_flag = (u8)ctx->param.chroma_ipf_enable_flag;
#endif
        // inter
#if UMVE_ENH
        sqh->umve_enh_enable_flag = (u8)ctx->param.umve_enh_enable_flag;
#endif
#if AFFINE_UMVE
#if SH_UNIFY
        sqh->affine_umve_enable_flag = (u8)ctx->param.affine_umve_enable_flag;
#else
        if (sqh->affine_enable_flag && sqh->umve_enable_flag && (u8)ctx->param.affine_umve_enable_flag)
        {
            sqh->affine_umve_enable_flag = 1;
        }
        else
        {
            sqh->affine_umve_enable_flag = 0;
        }
#endif
#endif
#if ASP
        sqh->asp_enable_flag = (u8)ctx->param.asp_enable_flag;
#endif
#if AWP
        if (ctx->param.awp_enable_flag)
        {
            sqh->awp_enable_flag = 1;
        }
        else
        {
            sqh->awp_enable_flag = 0;
        }
#endif
#if AWP_MVR
        sqh->awp_mvr_enable_flag = (u8)ctx->param.awp_mvr_enable_flag;
#endif
#if SUB_TMVP
        sqh->sbtmvp_enable_flag = (u8)ctx->param.sbtmvp_enable_flag;
#endif
#if SH_UNIFY
        sqh->emvp_enable_flag = 0;
#if ETMVP
        sqh->emvp_enable_flag = sqh->emvp_enable_flag || ctx->param.etmvp_enable_flag;
#endif
#if MVAP
        sqh->emvp_enable_flag = sqh->emvp_enable_flag || ctx->param.mvap_enable_flag;
#endif
#if ETMVP
        sqh->etmvp_enable_flag = (u8)sqh->emvp_enable_flag;
#endif
#if MVAP
        sqh->mvap_enable_flag = (u8)sqh->emvp_enable_flag;
        sqh->num_of_mvap_cand = sqh->mvap_enable_flag ? ALLOWED_MVAP_NUM : 0;
#endif
#else
#if MVAP
        sqh->mvap_enable_flag = (u8)ctx->param.mvap_enable_flag;
        sqh->num_of_mvap_cand = sqh->mvap_enable_flag ? ALLOWED_MVAP_NUM : 0;
#endif
#if ETMVP
        sqh->etmvp_enable_flag = (u8)ctx->param.etmvp_enable_flag;;
#endif
#endif

#if DMVR
        sqh->dmvr_enable_flag = (u8)ctx->param.dmvr_enable_flag;
#endif
#if BIO
        sqh->bio_enable_flag = (u8)ctx->param.bio_enable_flag;
#endif
#if BGC
        sqh->bgc_enable_flag = (u8)ctx->param.bgc_enable_flag;
#endif
#if INTERPF
        sqh->interpf_enable_flag = (u8)ctx->param.interpf_enable_flag;
#endif
#if IPC
        sqh->ipc_enable_flag = (u8)ctx->param.ipc_enable_flag;
#endif
#if OBMC
        sqh->obmc_enable_flag = (u8)ctx->param.obmc_enable_flag;
#endif

        // transform
#if EST
        sqh->est_enable_flag = (u8)ctx->param.est_enable_flag;
#endif
#if ST_CHROMA
        sqh->st_chroma_enable_flag = (u8)ctx->param.st_chroma_enable_flag;
#endif
#if SBT
        sqh->sbt_enable_flag = (u8)ctx->param.sbt_enable_flag;
#endif
#if IST
        sqh->ist_enable_flag = (u8)ctx->param.ist_enable_flag;
#endif
#if ISTS
#if SH_UNIFY
        // ists is always same with ist
        sqh->ists_enable_flag = sqh->ist_enable_flag;
#else
        sqh->ists_enable_flag = (u8)ctx->param.ists_enable_flag;
#endif
#endif
#if TS_INTER
#if SH_UNIFY
        // ts_inter is always same with ist
        sqh->ts_inter_enable_flag = sqh->ist_enable_flag;
#else
        sqh->ts_inter_enable_flag = (u8)ctx->param.ts_inter_enable_flag;
#endif
#endif

        // cabac
#if SRCC
        sqh->srcc_enable_flag = (u8)ctx->param.srcc_enable_flag;
#endif
#if CABAC_MULTI_PROB
        sqh->mcabac_enable_flag = (u8)ctx->param.mcabac_enable_flag;
#endif

        // filter
#if DBK_SCC
#if SH_UNIFY
        sqh->loop_filter_type_enable_flag = 1;
#else
        sqh->loop_filter_type_enable_flag = ctx->param.loop_filter_type_enable_flag;
#endif
#endif
#if DBR
#if SH_UNIFY
        sqh->dbr_enable_flag = 1;
#else
        sqh->dbr_enable_flag = ctx->param.dbr_enable_flag;
#endif
        if (sqh->dbr_enable_flag)
        {
            for (i = 0; i < 6; i++)
            {
                ctx->dbr_pic_param[i].dbr_horizontal_enabled = FALSE;
                ctx->dbr_pic_param[i].dbr_vertical_enabled = FALSE;
#if EDBR
                ctx->dbr_pic_param[i].dbr_fs0_horizontal_enabled = FALSE;
                ctx->dbr_pic_param[i].dbr_fs0_vertical_enabled = FALSE;
#endif
                ctx->dbr_pic_poc[i] = 0;
            }
        }
#endif
#if ESAO
        sqh->esao_enable_flag = (u8)ctx->param.esao_enable_flag;
#endif
#if CCSAO
        sqh->ccsao_enable_flag = (u8)ctx->param.ccsao_enable_flag;
#endif
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
        sqh->adaptive_leveling_filter_enhance_flag = (u8)ctx->param.adaptive_leveling_filter_enhance_flag;
#endif

        //SCC
#if USE_IBC
        if (ctx->param.use_ibc_flag)
        {
            sqh->ibc_flag = 1;
        }
        else
        {
            sqh->ibc_flag = 0;
        }
#endif
#if IBC_ABVR
        sqh->abvr_enable_flag = (u8)ctx->param.abvr_enable_flag;
#endif
#if USE_SP
        sqh->isc_enable_flag = (u8)ctx->param.sp_enable_flag || (u8)ctx->param.evs_ubvs_enable_flag;
#endif
#if IBC_BVP
        sqh->num_of_hbvp_cand = (u8)ctx->param.num_of_hbvp_cand;
#endif
#if FIMC
        if (ctx->param.fimc_enable_flag)
        {
            sqh->fimc_enable_flag = 1;
        }
        else
        {
            sqh->fimc_enable_flag = 0;
        }
#endif
#if NN_HOOK
        sqh->nn_tools_set_hook = 0x00;
#if NN_FILTER
        if (ctx->param.num_of_nnlf > 0)
        {
            sqh->nnlf_enable_flag = 1;
            sqh->num_of_nnlf = ctx->param.num_of_nnlf;
        }
        else
        {
            sqh->nnlf_enable_flag = 0;
            sqh->num_of_nnlf = 0;
        }
        sqh->nn_tools_set_hook = sqh->nn_tools_set_hook | (sqh->nnlf_enable_flag << 0);
#endif
#endif
    }
#endif

#if TRACE_REC // turn off SAO and ALF
    sqh->sample_adaptive_offset_enable_flag = 0;
    sqh->adaptive_leveling_filter_enable_flag = 0;
#if ESAO
    sqh->esao_enable_flag = 0;
#endif
#if CCSAO
    sqh->ccsao_enable_flag = 0;
#endif
#endif

    if (sqh->low_delay == 1)
    {
        sqh->output_reorder_delay = 0;
    }
    else
    {
        //for random access mode, GOP 16
        sqh->output_reorder_delay = com_tbl_log2[ctx->param.max_b_frames + 1];//log2(GopSize)
        assert(com_tbl_log2[ctx->param.max_b_frames + 1] != -1);
    }

#if PATCH
    sqh->patch_stable = ctx->patch->stable;
    sqh->patch_ref_colocated = ctx->patch->ref_colocated;
    sqh->cross_patch_loop_filter = ctx->patch->cross_patch_loop_filter;
#if BUGFIX_386
    assert(ctx->patch->width >= min(2, ctx->info.pic_width_in_lcu));
#endif
    if (ctx->patch->stable)
    {
        sqh->patch_uniform = ctx->patch->uniform;
        if (ctx->patch->uniform)
        {
            sqh->patch_width_minus1 = ctx->patch->width - 1;
            sqh->patch_height_minus1 = ctx->patch->height - 1;
        }
    }
#endif
}

typedef struct _QP_ADAPT_PARAM
{
    int qp_offset_layer;
    double qp_offset_model_offset;
    double qp_offset_model_scale;
} QP_ADAPT_PARAM;


#if RPL_GOP32
QP_ADAPT_PARAM qp_adapt_param_ra_gop32[8] =
{
    { -4,  0,      0 },
    { -2,  0,      0 },
    { -1, -4.0604, 0.154575 },
    {  1, -4.0604, 0.154575 },
    {  5, -4.8332, 0.17145 },
    {  7, -4.9668, 0.174975 },
    {  8, -5.9444, 0.225 },
    {  9, -5.9444, 0.225 }
};
#endif
QP_ADAPT_PARAM qp_adapt_param_ra[8] =
{
    { -4,  0,      0 },
#if QP_ADJUST_RA
    {  0,  0,      0 },
    {  2, -4.0604, 0.154575 },
#else
    {  1,  0,      0 },
    {  1, -4.0604, 0.154575 },
#endif
    {  5, -4.8332, 0.17145 },
    {  7, -4.9668, 0.174975 },
    {  8, -5.9444, 0.225 },
    {  9, -5.9444, 0.225 },
    { 10, -5.9444, 0.225 }
};

QP_ADAPT_PARAM qp_adapt_param_ld[8] =
{
    {-1,  0.0000, 0.0000},
    { 1,  0.0000, 0.0000},
    { 4, -6.5000, 0.2590},
    { 5, -6.5000, 0.2590},
    { 6, -6.5000, 0.2590},
    { 7, -6.5000, 0.2590},
    { 8, -6.5000, 0.2590},
    { 9, -6.5000, 0.2590},
};
#if HLS_RPL        //Implementation for selecting and assigning RPL0 & RPL1 candidates in the SPS to SH
static void select_assign_rpl_for_sh(ENC_CTX *ctx, COM_PIC_HEADER *pic_header)
{
    //TBD(@Chernyak) if the current picture is an IDR, simply return without doing the rest of the codes for this function
    int gop_num = ctx->param.gop_size;
    if ((ctx->param.max_b_frames == 0) || (ctx->param.gop_size == 1))
    {
        gop_num = 8;
    }

    //Assume it the pic is in the normal GOP first. Normal GOP here means it is not the first (few) GOP in the beginning of the bitstream
    pic_header->rpl_l0_idx = pic_header->rpl_l1_idx = -1;
    pic_header->ref_pic_list_sps_flag[0] = pic_header->ref_pic_list_sps_flag[1] = 0;
    for (int i = 0; i < MAX_NUM_REF_PICS; i++)
    {
        pic_header->rpl_l0.ref_pics[i] = 0;
        pic_header->rpl_l1.ref_pics[i] = 0;
        pic_header->rpl_l0.ref_pics_ddoi[i] = 0;
        pic_header->rpl_l1.ref_pics_ddoi[i] = 0;
        pic_header->rpl_l0.ref_pics_doi[i] = 0;
        pic_header->rpl_l1.ref_pics_doi[i] = 0;
#if LIBVC_ON
        pic_header->rpl_l0.library_index_flag[i] = 0;
        pic_header->rpl_l1.library_index_flag[i] = 0;
#endif
    }
    int availableRPLs = (ctx->param.rpls_l0_cfg_num < gop_num) ? ctx->param.rpls_l0_cfg_num : gop_num;
    for (int i = 0; i < availableRPLs; i++)
    {
        int pocIdx = (pic_header->poc % gop_num == 0) ? gop_num : pic_header->poc % gop_num;
        if (pocIdx == ctx->param.rpls_l0[i].poc)
        {
            pic_header->rpl_l0_idx = i;
            pic_header->rpl_l1_idx = pic_header->rpl_l0_idx;
            break;
        }
    }

    //For special case when the pic is in the first (few) GOP in the beginning of the bitstream.
    if (ctx->param.i_period <= 0)                                   // For low delay configuration
    {
        if (pic_header->poc <= (ctx->param.rpls_l0_cfg_num - gop_num))
        {
            pic_header->rpl_l0_idx = pic_header->poc + gop_num - 1;
            pic_header->rpl_l1_idx = pic_header->rpl_l0_idx;
        }
    }
    else                                                            // For random access configuration
    {
        for (int i = ctx->param.gop_size; i < ctx->param.rpls_l0_cfg_num; i++)
        {
            int pocIdx = (pic_header->poc % ctx->param.i_period == 0) ? ctx->param.i_period : pic_header->poc % ctx->param.i_period;
            if (pocIdx == ctx->param.rpls_l0[i].poc)
            {
                pic_header->rpl_l0_idx = i;
                pic_header->rpl_l1_idx = i;
                break;
            }
        }
    }

    // update slice type
    if (ctx->param.i_period == 0 && pic_header->poc != 0) // LD
    {
        assert(pic_header->rpl_l0_idx != -1);
        ctx->slice_type = ctx->param.rpls_l0[pic_header->rpl_l0_idx].slice_type;
        pic_header->slice_type = ctx->slice_type;
    }
    else if (ctx->param.i_period != 0 && pic_header->poc % ctx->param.i_period != 0) // RA
    {
        assert(pic_header->rpl_l0_idx != -1);
        ctx->slice_type = ctx->param.rpls_l0[pic_header->rpl_l0_idx].slice_type;
        pic_header->slice_type = ctx->slice_type;
    }

    // Copy RPL0 from the candidate in SPS to this SH
    pic_header->rpl_l0.poc = pic_header->poc;
    if (pic_header->rpl_l0_idx == -1)
    {
        pic_header->rpl_l0.tid = 0;
        pic_header->rpl_l0.ref_pic_num = 0;
        pic_header->rpl_l0.ref_pic_active_num = 0;
#if LIBVC_ON
        pic_header->rpl_l0.reference_to_library_enable_flag = 0;
#endif
    }
    else
    {
        pic_header->rpl_l0.tid = ctx->param.rpls_l0[pic_header->rpl_l0_idx].tid;
        pic_header->rpl_l0.ref_pic_num = ctx->param.rpls_l0[pic_header->rpl_l0_idx].ref_pic_num;
        pic_header->rpl_l0.ref_pic_active_num = ctx->param.rpls_l0[pic_header->rpl_l0_idx].ref_pic_active_num;
#if LIBVC_ON
        pic_header->rpl_l0.reference_to_library_enable_flag = ctx->param.rpls_l0[pic_header->rpl_l0_idx].reference_to_library_enable_flag;
#endif
    }
    for (int i = 0; i < pic_header->rpl_l0.ref_pic_num; i++)
    {
        pic_header->rpl_l0.ref_pics[i] = ctx->param.rpls_l0[pic_header->rpl_l0_idx].ref_pics[i];
        pic_header->rpl_l0.ref_pics_ddoi[i] = ctx->param.rpls_l0[pic_header->rpl_l0_idx].ref_pics_ddoi[i];
#if LIBVC_ON
        pic_header->rpl_l0.library_index_flag[i] = ctx->param.rpls_l0[pic_header->rpl_l0_idx].library_index_flag[i];
#endif
    }

    // Copy RPL1 from the candidate in SPS to this SH
    pic_header->rpl_l1.poc = pic_header->poc;
    if (pic_header->rpl_l1_idx == -1)
    {
        pic_header->rpl_l1.tid = 0;
        pic_header->rpl_l1.ref_pic_num = 0;
        pic_header->rpl_l1.ref_pic_active_num = 0;
#if LIBVC_ON
        pic_header->rpl_l1.reference_to_library_enable_flag = 0;
#endif
    }
    else
    {
        pic_header->rpl_l1.tid = ctx->param.rpls_l1[pic_header->rpl_l1_idx].tid;
        pic_header->rpl_l1.ref_pic_num = ctx->param.rpls_l1[pic_header->rpl_l1_idx].ref_pic_num;
        pic_header->rpl_l1.ref_pic_active_num = ctx->param.rpls_l1[pic_header->rpl_l1_idx].ref_pic_active_num;
#if LIBVC_ON
        pic_header->rpl_l1.reference_to_library_enable_flag = ctx->param.rpls_l1[pic_header->rpl_l1_idx].reference_to_library_enable_flag;
#endif
    }
    for (int i = 0; i < pic_header->rpl_l1.ref_pic_num; i++)
    {
        pic_header->rpl_l1.ref_pics[i] = ctx->param.rpls_l1[pic_header->rpl_l1_idx].ref_pics[i];
        pic_header->rpl_l1.ref_pics_ddoi[i] = ctx->param.rpls_l1[pic_header->rpl_l1_idx].ref_pics_ddoi[i];
#if LIBVC_ON
        pic_header->rpl_l1.library_index_flag[i] = ctx->param.rpls_l1[pic_header->rpl_l1_idx].library_index_flag[i];
#endif
    }

    /* update by Yuqun Fan, no profile level here
    if (sh->rpl_l0_idx != -1 && PROFILE_IS_MAIN(PROFILE))
    {
        sh->ref_pic_list_sps_flag[0] = 1;
    }

    if (sh->rpl_l1_idx != -1 && PROFILE_IS_MAIN(PROFILE))
    {
        sh->ref_pic_list_sps_flag[1] = 1;
    }
    */
    if (pic_header->rpl_l0_idx != -1)
    {
        pic_header->ref_pic_list_sps_flag[0] = 1;
    }

    if (pic_header->rpl_l1_idx != -1)
    {
        pic_header->ref_pic_list_sps_flag[1] = 1;
    }
}

//Return value 0 means all ref pic listed in the given rpl are available in the DPB
//Return value 1 means there is at least one ref pic listed in the given rpl not available in the DPB
static int check_refpic_available(int currentPOC, COM_PM *pm, COM_RPL *rpl)
{
    for (int i = 0; i < rpl->ref_pic_num; i++)
    {
        int isExistInDPB = 0;
        for (int j = 0; !isExistInDPB && j < MAX_PB_SIZE; j++)
        {
#if LIBVC_ON
            if (pm->pic[j] && pm->pic[j]->is_ref && pm->pic[j]->ptr == (currentPOC - rpl->ref_pics[i]) && !rpl->library_index_flag[i])
#else
            if (pm->pic[j] && pm->pic[j]->is_ref && pm->pic[j]->ptr == (currentPOC - rpl->ref_pics[i]))
#endif
                isExistInDPB = 1;
        }
        if (!isExistInDPB) //Found one ref pic missing return 1
            return 1;
    }
    return 0;
}

//Return value 0 means no explicit RPL is created. The given input parameters rpl0 and rpl1 are not modified
//Return value 1 means the given input parameters rpl0 and rpl1 are modified
static int create_explicit_rpl(COM_PM *pm, COM_PIC_HEADER *sh)

{
    int currentPOC = sh->poc;
    COM_RPL *rpl0 = &sh->rpl_l0;
    COM_RPL *rpl1 = &sh->rpl_l1;
    if (!check_refpic_available(currentPOC, pm, rpl0) && !check_refpic_available(currentPOC, pm, rpl1))
    {
        return 0;
    }

    COM_PIC * pic = NULL;

    int isRPLChanged = 0;
    //Remove ref pic in RPL0 that is not available in the DPB
    for (int ii = 0; ii < rpl0->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
#if LIBVC_ON
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl0->ref_pics[ii]) && !rpl0->library_index_flag[ii])
#else
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl0->ref_pics[ii]))
#endif
                isAvailable = 1;
            pic = NULL;
        }
        if (!isAvailable)
        {
            for (int jj = ii; jj < rpl0->ref_pic_num - 1; jj++)
            {
                rpl0->ref_pics[jj] = rpl0->ref_pics[jj + 1];
                rpl0->ref_pics_ddoi[jj] = rpl0->ref_pics_ddoi[jj + 1];
#if LIBVC_ON
                rpl0->library_index_flag[jj] = rpl0->library_index_flag[jj + 1];
#endif
            }
            ii--;
            rpl0->ref_pic_num--;
            isRPLChanged = 1;
        }
    }
    if (isRPLChanged)
        sh->rpl_l0_idx = -1;

    //Remove ref pic in RPL1 that is not available in the DPB
    isRPLChanged = 0;
    for (int ii = 0; ii < rpl1->ref_pic_num; ii++)
    {
        int isAvailable = 0;
        for (int jj = 0; !isAvailable && jj < pm->cur_num_ref_pics; jj++)
        {
            pic = pm->pic[jj];
#if LIBVC_ON
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl1->ref_pics[ii]) && !rpl1->library_index_flag[ii])
#else
            if (pic && pic->is_ref && pic->ptr == (currentPOC - rpl1->ref_pics[ii]))
#endif
                isAvailable = 1;
            pic = NULL;
        }
        if (!isAvailable)
        {
            for (int jj = ii; jj < rpl1->ref_pic_num - 1; jj++)
            {
                rpl1->ref_pics[jj] = rpl1->ref_pics[jj + 1];
                rpl1->ref_pics_ddoi[jj] = rpl1->ref_pics_ddoi[jj + 1];
#if LIBVC_ON
                rpl1->library_index_flag[jj] = rpl1->library_index_flag[jj + 1];
#endif
            }
            ii--;
            rpl1->ref_pic_num--;
            isRPLChanged = 1;
        }
    }
    if (isRPLChanged)
        sh->rpl_l1_idx = -1;

    /*if number of ref pic in RPL0 is less than its number of active ref pic, try to copy from RPL1*/
    if (rpl0->ref_pic_num < rpl0->ref_pic_active_num)
    {
        for (int ii = rpl0->ref_pic_num; ii < rpl0->ref_pic_active_num; ii++)
        {
            //First we need to find ref pic in RPL1 that is not already in RPL0
            int isAlreadyIncluded = 1;
            int idx = -1;
            int status = 0;
            do
            {
                status = 0;
                idx++;
                for (int mm = 0; mm < rpl0->ref_pic_num && idx < rpl1->ref_pic_num; mm++)
                {
                    if (rpl1->ref_pics[idx] == rpl0->ref_pics[mm])
                        status = 1;
                }
                if (!status) isAlreadyIncluded = 0;
            }
            while (isAlreadyIncluded && idx < rpl1->ref_pic_num);

            if (idx < rpl1->ref_pic_num)
            {
                rpl0->ref_pics[ii] = rpl1->ref_pics[idx];
                rpl0->ref_pics_ddoi[ii] = rpl1->ref_pics_ddoi[idx];
#if LIBVC_ON
                rpl0->library_index_flag[ii] = rpl1->library_index_flag[idx];
#endif
                rpl0->ref_pic_num++;
            }
        }
        if (rpl0->ref_pic_num < rpl0->ref_pic_active_num) rpl0->ref_pic_active_num = rpl0->ref_pic_num;
    }

    /*same logic as above, just apply to RPL1*/
    if (rpl1->ref_pic_num < rpl1->ref_pic_active_num)
    {
        for (int ii = rpl1->ref_pic_num; ii < rpl1->ref_pic_active_num; ii++)
        {
            int isAlreadyIncluded = 1;
            int idx = -1;
            int status = 0;
            do
            {
                status = 0;
                idx++;
                for (int mm = 0; mm < rpl1->ref_pic_num && idx < rpl0->ref_pic_num; mm++)
                {
                    if (rpl0->ref_pics[idx] == rpl1->ref_pics[mm])
                        status = 1;
                }
                if (!status) isAlreadyIncluded = 0;
            }
            while (isAlreadyIncluded && idx < rpl0->ref_pic_num);

            if (idx < rpl0->ref_pic_num)
            {
                rpl1->ref_pics[ii] = rpl0->ref_pics[idx];
                rpl1->ref_pics_ddoi[ii] = rpl0->ref_pics_ddoi[idx];
#if LIBVC_ON
                rpl1->library_index_flag[ii] = rpl0->library_index_flag[idx];
#endif
                rpl1->ref_pic_num++;
            }
        }
        if (rpl1->ref_pic_num < rpl1->ref_pic_active_num) rpl1->ref_pic_active_num = rpl1->ref_pic_num;
    }
    return 1;
}
#endif
static void set_sh(ENC_CTX *ctx, COM_SH_EXT *shext, s8 *sao_enable_patch)
{
    shext->fixed_slice_qp_flag = (u8)ctx->info.pic_header.fixed_picture_qp_flag;
    shext->slice_qp = (u8)ctx->info.pic_header.picture_qp;
    shext->slice_sao_enable[Y_C] = sao_enable_patch[Y_C];
    shext->slice_sao_enable[U_C] = sao_enable_patch[U_C];
    shext->slice_sao_enable[V_C] = sao_enable_patch[V_C];
}

#if DBR
void dbr_param_copy(DBR_PARAM *dbr_param_dst, DBR_PARAM *dbr_param_src)
{
    dbr_param_dst->dbr_horizontal_enabled = dbr_param_src->dbr_horizontal_enabled;
    dbr_param_dst->thresh_horizontal_index = dbr_param_src->thresh_horizontal_index;
    dbr_param_dst->horizontal_offsets[0] = dbr_param_src->horizontal_offsets[0];
    dbr_param_dst->horizontal_offsets[1] = dbr_param_src->horizontal_offsets[1];

    dbr_param_dst->dbr_vertical_enabled = dbr_param_src->dbr_vertical_enabled;
    dbr_param_dst->thresh_vertical_index = dbr_param_src->thresh_vertical_index;
    dbr_param_dst->vertical_offsets[0] = dbr_param_src->vertical_offsets[0];
    dbr_param_dst->vertical_offsets[1] = dbr_param_src->vertical_offsets[1];

#if EDBR
    dbr_param_dst->dbr_fs0_horizontal_enabled = dbr_param_src->dbr_fs0_horizontal_enabled;
    dbr_param_dst->horizontal_offsets[0 + 6] = dbr_param_src->horizontal_offsets[0 + 6];
    dbr_param_dst->horizontal_offsets[1 + 6] = dbr_param_src->horizontal_offsets[1 + 6];

    dbr_param_dst->dbr_fs0_vertical_enabled = dbr_param_src->dbr_fs0_vertical_enabled;
    dbr_param_dst->vertical_offsets[0 + 6] = dbr_param_src->vertical_offsets[0 + 6];
    dbr_param_dst->vertical_offsets[1 + 6] = dbr_param_src->vertical_offsets[1 + 6];
#endif
}
#endif

static void set_pic_header(ENC_CTX *ctx, COM_PIC_HEADER *pic_header)
{
    double qp;
#if DBR
    int pic_idx;
    int h_offset[4] = { 0 };  // two offsets for two thresholds
    int v_offset[4] = { 0 };
    int h_cnt[2]    = { 0 };  // counters for for two thresholds
    int v_cnt[2]    = { 0 };
#if EDBR
    int h_offset_fs0[4] = { 0 };  // two offsets for two thresholds
    int v_offset_fs0[4] = { 0 };
    int h_cnt_fs0[2] = { 0 };  // counters for for two thresholds
    int v_cnt_fs0[2] = { 0 };
#endif
#endif
#if LIBVC_ON
    QP_ADAPT_PARAM *qp_adapt_param;
    if (ctx->rpm.libvc_data->is_libpic_processing)
    {
        qp_adapt_param = qp_adapt_param_ra;
    }
    else
    {
#if RPL_GOP32
        if (ctx->param.max_b_frames == 0)
        {
            qp_adapt_param = qp_adapt_param_ld;
        }
        else if (ctx->param.max_b_frames == 31)
        {
            qp_adapt_param = qp_adapt_param_ra_gop32;
        }
        else
        {
            qp_adapt_param = qp_adapt_param_ra;
        }
#else
        qp_adapt_param = ctx->param.max_b_frames == 0 ? qp_adapt_param_ld : qp_adapt_param_ra;
#endif
    }
#else
    QP_ADAPT_PARAM *qp_adapt_param = ctx->param.max_b_frames == 0 ? qp_adapt_param_ld : qp_adapt_param_ra;
#endif
    pic_header->bbv_delay =  0xFFFFFFFF;
    pic_header->time_code_flag = 0;
    pic_header->time_code = 0;
    pic_header->low_delay = ctx->info.sqh.low_delay;
    if (pic_header->low_delay)
    {
        pic_header->bbv_check_times = (1 << 16) - 2;
    }
    else
    {
        pic_header->bbv_check_times = 0;
    }

    pic_header->dtr = ctx->dtr;
    pic_header->slice_type = ctx->slice_type;
#if TRACE_REC // turn off Deblock
    pic_header->loop_filter_disable_flag = 1;
#else
    pic_header->loop_filter_disable_flag = (ctx->param.use_deblock) ? 0 : 1;
#endif

#if DBR
    pic_header->ph_dbr_param.dbr_vertical_enabled    = FALSE;
    pic_header->ph_dbr_param.dbr_horizontal_enabled  = FALSE;
#if SH_UNIFY
#if EDBR
    pic_header->ph_dbr_param.dbr_fs0_vertical_enabled    = FALSE;
    pic_header->ph_dbr_param.dbr_fs0_horizontal_enabled  = FALSE;
#endif
    if (ctx->info.sqh.dbr_enable_flag && ctx->param.dbr_enable_flag) // control by cfg and profile
#else
    if (ctx->info.sqh.dbr_enable_flag )
#endif
    {        
#if EDBR
        int fs0_off = ((ctx->param.i_period > 1 && ctx->param.pic_height <= 720 && !(pic_header->slice_type == SLICE_I && ctx->param.qp <= 27)) || (ctx->param.i_period == 0 && ctx->param.pic_height <= 1080 && !(pic_header->slice_type == SLICE_I && ctx->param.qp >= 45)));
        if (ctx->param.i_period > 1 && (pic_header->slice_type == SLICE_I || 1 == (ctx->pic_cnt % ctx->param.i_period)))
#else
        if (pic_header->slice_type == SLICE_I || (ctx->param.i_period > 1 &&  1 == (ctx->pic_cnt % ctx->param.i_period)))
#endif
        {
            for (pic_idx = 0; pic_idx < 6; pic_idx++)
            {
                ctx->dbr_pic_param[pic_idx].dbr_horizontal_enabled = FALSE;
                ctx->dbr_pic_param[pic_idx].dbr_vertical_enabled = FALSE;
#if EDBR
                ctx->dbr_pic_param[pic_idx].dbr_fs0_horizontal_enabled = FALSE;
                ctx->dbr_pic_param[pic_idx].dbr_fs0_vertical_enabled   = FALSE;
#endif
                ctx->dbr_pic_poc[pic_idx] = 0;
            }
#if EDBR
            dbr_param_copy(&(pic_header->ph_dbr_param), &(ctx->dbr_pic_param[0]));
#endif
        }
        else
        {
#if EDBR
            if (pic_header->slice_type == SLICE_I)
            {
                dbr_param_copy(&(pic_header->ph_dbr_param), &(ctx->dbr_pic_param[0]));
            }
            else
            {
#endif
            for (pic_idx = 0; pic_idx < 6; pic_idx++)
            {
                if (ctx->dbr_pic_param[pic_idx].dbr_vertical_enabled)
                {
                    if (1 == ctx->dbr_pic_param[pic_idx].thresh_vertical_index)
                    {
                        v_offset[0] += ctx->dbr_pic_param[pic_idx].vertical_offsets[0];
                        v_offset[1] += ctx->dbr_pic_param[pic_idx].vertical_offsets[1];
                        v_cnt[0]    ++;
                    }
                    else
                    {
                        v_offset[2] += ctx->dbr_pic_param[pic_idx].vertical_offsets[0];
                        v_offset[3] += ctx->dbr_pic_param[pic_idx].vertical_offsets[1];
                        v_cnt[1]    ++;
                    }
                }

                if (ctx->dbr_pic_param[pic_idx].dbr_horizontal_enabled)
                {
                    if (1 == ctx->dbr_pic_param[pic_idx].thresh_horizontal_index)
                    {
                        h_offset[0] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[0];
                        h_offset[1] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[1];
                        h_cnt[0]    ++;
                    }
                    else
                    {
                        h_offset[2] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[0];
                        h_offset[3] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[1];
                        h_cnt[1]    ++;
                    }
                }
#if EDBR
                if (ctx->dbr_pic_param[pic_idx].dbr_fs0_vertical_enabled)
                {
                    if (1 == ctx->dbr_pic_param[pic_idx].thresh_vertical_index)
                    {
                        v_offset_fs0[0] += ctx->dbr_pic_param[pic_idx].vertical_offsets[0 + 6];
                        v_offset_fs0[1] += ctx->dbr_pic_param[pic_idx].vertical_offsets[1 + 6];
                        v_cnt_fs0[0]    ++;
                    }
                    else
                    {
                        v_offset_fs0[2] += ctx->dbr_pic_param[pic_idx].vertical_offsets[0 + 6];
                        v_offset_fs0[3] += ctx->dbr_pic_param[pic_idx].vertical_offsets[1 + 6];
                        v_cnt_fs0[1]    ++;
                    }
                }

                if (ctx->dbr_pic_param[pic_idx].dbr_horizontal_enabled)
                {
                    if (1 == ctx->dbr_pic_param[pic_idx].thresh_horizontal_index)
                    {
                        h_offset_fs0[0] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[0 + 6];
                        h_offset_fs0[1] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[1 + 6];
                        h_cnt_fs0[0]    ++;
                    }
                    else
                    {
                        h_offset_fs0[2] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[0 + 6];
                        h_offset_fs0[3] += ctx->dbr_pic_param[pic_idx].horizontal_offsets[1 + 6];
                        h_cnt_fs0[1]    ++;
                    }
                }
#endif
            }
                if (v_cnt[0] || v_cnt[1])
                {
                    ctx->info.pic_header.ph_dbr_param.dbr_vertical_enabled = TRUE;
                    if (v_cnt[0] >= v_cnt[1])
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_vertical_index = 1;
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[0] = v_offset[0] / v_cnt[0];
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[1] = v_offset[1] / v_cnt[0];
                    }
                    else
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_vertical_index = 2;
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[0] = v_offset[2] / v_cnt[1];
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[1] = v_offset[3] / v_cnt[1];
                    }
                }

#if EDBR
                if (!fs0_off && (v_cnt_fs0[0] || v_cnt_fs0[1]))
                {
                    ctx->info.pic_header.ph_dbr_param.dbr_fs0_vertical_enabled = TRUE;
                    if (v_cnt_fs0[0] >= v_cnt_fs0[1])
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_vertical_index = 1;
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[0 + 6] = v_offset_fs0[0] / v_cnt_fs0[0];
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[1 + 6] = v_offset_fs0[1] / v_cnt_fs0[0];
                    }
                    else
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_vertical_index = 2;
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[0 + 6] = v_offset_fs0[2] / v_cnt_fs0[1];
                        ctx->info.pic_header.ph_dbr_param.vertical_offsets[1 + 6] = v_offset_fs0[3] / v_cnt_fs0[1];
                    }
                }
#endif

                if (h_cnt[0] || h_cnt[1])
                {
                    ctx->info.pic_header.ph_dbr_param.dbr_horizontal_enabled = TRUE;
                    if (h_cnt[0] >= h_cnt[1])
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_horizontal_index = 1;
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[0] = h_offset[0] / h_cnt[0];
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[1] = h_offset[1] / h_cnt[0];
                    }
                    else
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_horizontal_index = 2;
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[0] = h_offset[2] / h_cnt[1];
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[1] = h_offset[3] / h_cnt[1];
                    }
                }
#if EDBR
                if (!fs0_off && (h_cnt_fs0[0] || h_cnt_fs0[1]))
                {
                    ctx->info.pic_header.ph_dbr_param.dbr_fs0_horizontal_enabled = TRUE;
                    if (h_cnt_fs0[0] >= h_cnt_fs0[1])
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_horizontal_index = 1;
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[0 + 6] = h_offset_fs0[0] / h_cnt_fs0[0];
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[1 + 6] = h_offset_fs0[1] / h_cnt_fs0[0];
                    }
                    else
                    {
                        ctx->info.pic_header.ph_dbr_param.thresh_horizontal_index = 2;
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[0 + 6] = h_offset_fs0[2] / h_cnt_fs0[1];
                        ctx->info.pic_header.ph_dbr_param.horizontal_offsets[1 + 6] = h_offset_fs0[3] / h_cnt_fs0[1];
                    }
                }
            }
            if (fs0_off)
            {
                pic_header->ph_dbr_param.dbr_fs0_vertical_enabled = FALSE;
                pic_header->ph_dbr_param.dbr_fs0_horizontal_enabled = FALSE;
            }
#endif
        }
    }
#endif

    pic_header->temporal_id = ctx->temporal_id;

    if (ctx->info.sqh.wq_enable)
    {
        pic_header->pic_wq_enable = 1;//default
        pic_header->pic_wq_data_idx = ctx->param.pic_wq_data_idx;
        if (pic_header->pic_wq_data_idx == 0)
        {
            memcpy(pic_header->wq_4x4_matrix, ctx->info.sqh.wq_4x4_matrix, sizeof(pic_header->wq_4x4_matrix));
            memcpy(pic_header->wq_8x8_matrix, ctx->info.sqh.wq_8x8_matrix, sizeof(pic_header->wq_8x8_matrix));
        }
        else if (pic_header->pic_wq_data_idx == 1)
        {
            pic_header->wq_param = ctx->param.wq_param;
            pic_header->wq_model = ctx->param.wq_model;

            if (pic_header->wq_param == 0)
            {
                memcpy(pic_header->wq_param_vector, tab_wq_param_default[1], sizeof(pic_header->wq_param_vector));
            }
            else if (pic_header->wq_param == 1)
            {
                sscanf(ctx->param.wq_param_undetailed, "[%d,%d,%d,%d,%d,%d]",
                       &pic_header->wq_param_vector[0],
                       &pic_header->wq_param_vector[1],
                       &pic_header->wq_param_vector[2],
                       &pic_header->wq_param_vector[3],
                       &pic_header->wq_param_vector[4],
                       &pic_header->wq_param_vector[5]);
            }
            else
            {
                sscanf(ctx->param.wq_param_detailed, "[%d,%d,%d,%d,%d,%d]",
                       &pic_header->wq_param_vector[0],
                       &pic_header->wq_param_vector[1],
                       &pic_header->wq_param_vector[2],
                       &pic_header->wq_param_vector[3],
                       &pic_header->wq_param_vector[4],
                       &pic_header->wq_param_vector[5]);
            }
            set_pic_wq_matrix_by_param(pic_header->wq_param_vector, pic_header->wq_model, pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
        }
        else
        {
            load_wq_matrix(ctx->param.pic_wq_user, pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
        }
    }
    else
    {
        pic_header->pic_wq_enable = 0;
        init_pic_wq_matrix(pic_header->wq_4x4_matrix, pic_header->wq_8x8_matrix);
    }

#if HLS_RPL
    pic_header->poc = ctx->ptr;
    select_assign_rpl_for_sh(ctx, pic_header);
    pic_header->num_ref_idx_active_override_flag = 1;
    if (pic_header->slice_type == SLICE_I)
    {
        pic_header->rpl_l0.ref_pic_active_num = 0;
        pic_header->rpl_l1.ref_pic_active_num = 0;
    }
    //Hendry -- @Roman: this value should be derived based on the value of pic_header->rpl_l0.ref_pic_active_num and num_ref_idx_default_active_minus1[ 0 ]. Also pic_header->rpl_l1.ref_pic_active_num and num_ref_idx_default_active_minus1[ 1 ]
    //If (pic_header->rpl_l0.ref_pic_active_num == num_ref_idx_default_active_minus1[ 0 ]) && (pic_header->rpl_l1.ref_pic_active_num and num_ref_idx_default_active_minus1[ 1 ]) then pic_header->num_ref_idx_active_override_flag = 0 else 1
#endif

    pic_header->progressive_frame  = 1;
    pic_header->picture_structure  = 1;
    pic_header->top_field_first    = 0;
    pic_header->repeat_first_field = 0;
    pic_header->top_field_picture_flag = 0;

    pic_header->fixed_picture_qp_flag = !ctx->param.delta_qp_flag;
#if CUDQP
    pic_header->cu_delta_qp_flag = !pic_header->fixed_picture_qp_flag && ctx->param.cu_delta_qp_flag;
    pic_header->cu_qp_group_size = ctx->param.cu_qp_group_size;
    pic_header->cu_qp_group_area_size = pic_header->cu_qp_group_size * pic_header->cu_qp_group_size;
#endif
    pic_header->random_access_decodable_flag = 1; // the value shall be 0 for leading pictures according to text (however, not used in decoder)
#if LIBVC_ON
    pic_header->is_RLpic_flag = ctx->is_RLpic_flag;
    if (ctx->info.sqh.library_stream_flag)
    {
        pic_header->library_picture_index = ctx->ptr;
    }
    else
    {
        pic_header->library_picture_index = -1;
    }
#endif
    // Here the input QP is converted to be the encoding QP.
    qp = ctx->param.qp + ctx->info.qp_offset_bit_depth;
#if LIBVC_ON
    if (ctx->param.qp_offset_rlpic)
    {
        assert(ctx->param.frame_qp_add == 0);
        qp += pic_header->is_RLpic_flag == 1 ? ctx->param.qp_offset_rlpic : 0;
    }
#endif
    if( ctx->param.pb_frame_qp_offset )
    {
        //to simulate IPPP structure with P frame QP = I frame QP + pb_frame_qp_offset
        assert( ctx->param.frame_qp_add == 0 );
        assert( ctx->param.disable_hgop == 1 );
        qp += pic_header->slice_type != SLICE_I ? ctx->param.pb_frame_qp_offset : 0;
    }
    //add one qp after certain frame (use to match target bitrate for fix qp case, e.g., Cfp bitrate matching)
    qp = COM_CLIP3(MIN_QUANT, (MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth), (ctx->param.frame_qp_add != 0 && (int)(ctx->ptr) >= ctx->param.frame_qp_add) ? qp + 1.0 : qp);

    if(!ctx->param.disable_hgop)
    {
        double dqp_offset;
        int qp_offset;
        qp += qp_adapt_param[ctx->temporal_id].qp_offset_layer;
        dqp_offset = COM_MAX(0, qp - ctx->info.qp_offset_bit_depth) * qp_adapt_param[ctx->temporal_id].qp_offset_model_scale + qp_adapt_param[ctx->temporal_id].qp_offset_model_offset + 0.5;
        qp_offset = (int)floor(COM_CLIP3(0.0, 4.0, dqp_offset));
        qp += qp_offset;
    }

    pic_header->picture_qp = (u8)COM_CLIP3(MIN_QUANT, (MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth), qp);

#if HDR_CHROMA_QP_OFFSET
    if (ctx->param.hdr_chroma_qp_offset)
    {
        double dcbQP;
        int cbQP, crQP;
        double dcrQP;
        dcbQP = -0.1141*ctx->param.qp - 0.8831;
        cbQP = (int)(dcbQP + (dcbQP < 0 ? -0.5 : 0.5));
        cbQP = COM_CLIP(cbQP, -6, 6);
        dcrQP = (double)ctx->param.qp/8;
        if (dcrQP>5)
            crQP = -6;
        else
            crQP = -5;
        crQP = COM_CLIP(crQP, -6, 6);
        pic_header->chroma_quant_param_delta_cb = COM_CLIP((min(0, cbQP) + ctx->param.qp_offset_cb),-6,6);
        pic_header->chroma_quant_param_delta_cr = COM_CLIP((min(0, crQP) + ctx->param.qp_offset_cr),-6,6);
    }
    else if (ctx->param.qp_offset_adp)
#else
    if (ctx->param.qp_offset_adp)
#endif
    {
        //find a qp_offset_cb that makes com_tbl_qp_chroma_adjust[qp + qp_offset_cb] equal to com_tbl_qp_chroma_adjust_enc[qp + 1]
        int adaptive_qp_offset_encoder;
        int target_chroma_qp = com_tbl_qp_chroma_adjust_enc[COM_CLIP(pic_header->picture_qp - ctx->info.qp_offset_bit_depth + 1, 0, 63)];
        for (adaptive_qp_offset_encoder = -5; adaptive_qp_offset_encoder < 10; adaptive_qp_offset_encoder++)
        {
            if (target_chroma_qp == com_tbl_qp_chroma_adjust[COM_CLIP(pic_header->picture_qp - ctx->info.qp_offset_bit_depth + adaptive_qp_offset_encoder, 0, 63)])
            {
                break;
            }
        }
        if (pic_header->picture_qp - ctx->info.qp_offset_bit_depth + adaptive_qp_offset_encoder > 63)
        {
            adaptive_qp_offset_encoder = 63 - (pic_header->picture_qp - ctx->info.qp_offset_bit_depth);
        }
        pic_header->chroma_quant_param_delta_cb = ctx->param.qp_offset_cb + adaptive_qp_offset_encoder;
        pic_header->chroma_quant_param_delta_cr = ctx->param.qp_offset_cr + adaptive_qp_offset_encoder;
    }
    else
    {
        pic_header->chroma_quant_param_delta_cb = ctx->param.qp_offset_cb;
        pic_header->chroma_quant_param_delta_cr = ctx->param.qp_offset_cr;
    }

    pic_header->loop_filter_parameter_flag = 0;
    pic_header->alpha_c_offset = 0;
    pic_header->beta_offset = 0;
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
    pic_header->tool_alf_enhance_on = ctx->info.sqh.adaptive_leveling_filter_enhance_flag;
#endif
    pic_header->tool_alf_on = ctx->info.sqh.adaptive_leveling_filter_enable_flag;

    pic_header->affine_subblock_size_idx = 0; // 0->4X4, 1->8X8

    if (pic_header->chroma_quant_param_delta_cb == 0 && pic_header->chroma_quant_param_delta_cr == 0)
    {
        pic_header->chroma_quant_param_disable_flag = 1;
    }
    else
    {
        pic_header->chroma_quant_param_disable_flag = 0;
    }
    pic_header->picture_output_delay = ctx->ptr - ctx->info.pic_header.decode_order_index + ctx->info.sqh.output_reorder_delay;
}

static int enc_eco_tree(ENC_CTX * ctx, ENC_CORE * core, int x0, int y0, int cup, int cu_width, int cu_height, int cud
                        , const int parent_split, int qt_depth, int bet_depth, u8 cons_pred_mode, u8 tree_status)
{
    int ret;
    COM_BSW * bs;
    s8 split_mode;
    com_get_split_mode(&split_mode, cud, cup, cu_width, cu_height, ctx->info.max_cuwh, ctx->map_cu_data[core->lcu_num].split_mode);
#if CHROMA_NOT_SPLIT
    u8 tree_status_child = TREE_LC;
#endif
#if MODE_CONS
    u8 cons_pred_mode_child;
#endif
    bs = &ctx->bs;

    if (cu_width > MIN_CU_SIZE || cu_height > MIN_CU_SIZE)
    {
        COM_TRACE_COUNTER;
        COM_TRACE_STR("x pos ");
        COM_TRACE_INT(core->x_pel + ((cup % (ctx->info.max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
        COM_TRACE_STR("y pos ");
        COM_TRACE_INT(core->y_pel + ((cup / (ctx->info.max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
        COM_TRACE_STR("width ");
        COM_TRACE_INT(cu_width);
        COM_TRACE_STR("height ");
        COM_TRACE_INT(cu_height);
        COM_TRACE_STR("depth ");
        COM_TRACE_INT(cud);
        COM_TRACE_STR("\n");
    }
#if CUDQP
    if (com_is_cu_dqp(&ctx->info) && cu_width * cu_height >= ctx->info.pic_header.cu_qp_group_area_size)
    {
        assert(tree_status != TREE_C);
        ctx->cu_qp_group.num_delta_qp = 0;
        ctx->cu_qp_group.cu_qp_group_x = x0;
        ctx->cu_qp_group.cu_qp_group_y = y0;
        int luma_scup = PEL2SCU(x0) + PEL2SCU(y0) * ctx->info.pic_width_in_scu;
        ctx->cu_qp_group.pred_qp = (x0 > 0 && MCU_GET_CODED_FLAG(ctx->map.map_scu[luma_scup - 1])) ? MCU_GET_QP(ctx->map.map_scu[luma_scup - 1]) : ctx->info.shext.slice_qp;
        assert(ctx->cu_qp_group.pred_qp >= 0 && ctx->cu_qp_group.pred_qp <= MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth);
    }
#endif
    int rsize = ctx->info.log2_max_cuwh < 7 ? (1 << ctx->info.log2_max_cuwh) : 64;
    if (x0 % rsize == 0 && y0 % rsize == 0 && cu_width == rsize && cu_height == rsize)
    {
        for (int i = 0; i < core->n_pv_num; i++)
        {
            if (!is_pv_valid(x0, y0, core->n_recent_pv_x[i], core->n_recent_pv_y[i] % (1 << ctx->info.log2_max_cuwh), ctx->info.log2_max_cuwh, ctx->info.pic_width_in_scu, ctx->map.map_scu, ctx->info.pic_width))
            {
                core->n_recent_dpb_idx[i] = 1;
            }
        }
    }
    if(split_mode != NO_SPLIT)
    {
        enc_eco_split_mode(bs, ctx, core, cud, cup, cu_width, cu_height, ctx->info.max_cuwh
                           , parent_split, qt_depth, bet_depth, x0, y0);
#if CHROMA_NOT_SPLIT
        tree_status_child = (tree_status == TREE_LC && com_tree_split(cu_width, cu_height, split_mode, ctx->info.pic_header.slice_type)) ? TREE_L : tree_status;
#endif
#if MODE_CONS
        if (cons_pred_mode == NO_MODE_CONS && com_constrain_pred_mode(cu_width, cu_height, split_mode, ctx->info.pic_header.slice_type))
        {
            cons_pred_mode_child = com_get_cons_pred_mode(cud, cup, cu_width, cu_height, ctx->info.max_cuwh, ctx->map_cu_data[core->lcu_num].split_mode);
            enc_eco_cons_pred_mode_child(bs, cons_pred_mode_child);
        }
        else
        {
            cons_pred_mode_child = cons_pred_mode;
        }
#endif
        COM_SPLIT_STRUCT split_struct;
        com_split_get_part_structure(split_mode, x0, y0, cu_width, cu_height, cup, cud, ctx->log2_culine, &split_struct);
        for (int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = part_num;
            int sub_cuw = split_struct.width[cur_part_num];
            int sub_cuh = split_struct.height[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if(x_pos < ctx->info.pic_width && y_pos < ctx->info.pic_height)
            {
                ret = enc_eco_tree(ctx, core, x_pos, y_pos, split_struct.cup[cur_part_num], sub_cuw, sub_cuh, split_struct.cud
                                   , split_mode, INC_QT_DEPTH(qt_depth, split_mode), INC_BET_DEPTH(bet_depth, split_mode), cons_pred_mode_child, tree_status_child);
                com_assert_g(COM_SUCCEEDED(ret), ERR);
            }
        }
#if CHROMA_NOT_SPLIT
        if (tree_status_child == TREE_L && tree_status == TREE_LC)
        {
            com_assert(x0 + cu_width <= ctx->info.pic_width && y0 + cu_height <= ctx->info.pic_height);
            ctx->tree_status = TREE_C;
            ctx->cons_pred_mode = NO_MODE_CONS;
            ret = enc_eco_unit_chroma(ctx, core, x0, y0, cup, cu_width, cu_height);
            ctx->tree_status = TREE_LC;
            int ui_pvbuf_size_prev = core->n_pv_num;
            {
                for (int i = 0;i < ui_pvbuf_size_prev;i++)
                {
                    int idx = i;

                    if (core->n_recent_cuS_flag[idx])
                    {
                        ENC_CU_DATA *cu_data = &ctx->map_cu_data[core->lcu_num];
                        core->n_recent_pv[1][idx] = cu_data->srb_u[idx + cup * MAX_SRB_SIZE];
                        core->n_recent_pv[2][idx] = cu_data->srb_v[idx + cup * MAX_SRB_SIZE];
                        core->n_recent_cuS_flag[idx] = 0;
                        assert(core->n_recent_pv[1][idx] < 1025 && core->n_recent_pv[1][idx] >= 0);
                        assert(core->n_recent_pv[2][idx] < 1025 && core->n_recent_pv[2][idx] >= 0);
                    }
                }
            }
        }
#endif
    }
    else
    {
        com_assert(x0 + cu_width <= ctx->info.pic_width && y0 + cu_height <= ctx->info.pic_height);
        enc_eco_split_mode(bs, ctx, core, cud, cup, cu_width, cu_height, ctx->info.max_cuwh
                           , parent_split, qt_depth, bet_depth, x0, y0);
#if CHROMA_NOT_SPLIT
        ctx->tree_status = tree_status;
#endif
#if MODE_CONS
        ctx->cons_pred_mode = cons_pred_mode;
#endif
        ret = enc_eco_unit(ctx, core, x0, y0, cup, cu_width, cu_height);
        com_assert_g(COM_SUCCEEDED(ret), ERR);
#if CUDQP
        assert(tree_status != TREE_C);
        if (com_is_cu_dqp(&ctx->info) && is_cu_nz(core->mod_info_curr.num_nz))
        {
            ctx->cu_qp_group.num_delta_qp++;
            ctx->cu_qp_group.pred_qp = core->qp_y;
        }
#endif
    }
    return COM_OK;
ERR:
    return ret;
}

int enc_ready(ENC_CTX * ctx)
{
    ENC_CORE * core = NULL;
    int ret, i;
    int pic_width = ctx->info.pic_width = ctx->param.pic_width;
    int pic_height = ctx->info.pic_height = ctx->param.pic_height;
    s64          size;
    com_assert(ctx);
    core = core_alloc();
    com_assert_gv(core != NULL, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    /* set various value */
    ctx->core           = core;

    enc_init_bits_est();
    ctx->info.max_cuwh          = ctx->param.ctu_size;
    ctx->info.log2_max_cuwh     = CONV_LOG2(ctx->info.max_cuwh);
    ctx->max_cud                = (u8)ctx->info.log2_max_cuwh - MIN_CU_LOG2;
    ctx->info.pic_width_in_lcu  = (pic_width + ctx->info.max_cuwh - 1) >> ctx->info.log2_max_cuwh;
    ctx->info.pic_height_in_lcu = (pic_height + ctx->info.max_cuwh - 1) >> ctx->info.log2_max_cuwh;
    ctx->info.f_lcu             = ctx->info.pic_width_in_lcu * ctx->info.pic_height_in_lcu;
    ctx->info.pic_width_in_scu  = (pic_width + ((1<<MIN_CU_LOG2)-1))>>MIN_CU_LOG2;
    ctx->info.pic_height_in_scu = (pic_height + ((1<<MIN_CU_LOG2)-1))>>MIN_CU_LOG2;
    ctx->info.f_scu             = ctx->info.pic_width_in_scu * ctx->info.pic_height_in_scu;
    ctx->log2_culine            = (u8)ctx->info.log2_max_cuwh - MIN_CU_LOG2;
    ctx->log2_cudim             = ctx->log2_culine << 1;

#if ASP
    ctx->info.skip_me_asp = FALSE;
    ctx->info.skip_umve_asp = FALSE;
#endif
#if USE_IBC
#if FIMC
#if ISTS
    if (ctx->param.use_ibc_flag || ctx->param.fimc_enable_flag == 1 || ctx->param.ists_enable_flag == 1
#if TS_INTER
        || ctx->param.ts_inter_enable_flag == 1
#endif
#if DBK_SCC
        || ctx->param.loop_filter_type_enable_flag == 1
#endif
#if AWP
        || ctx->param.awp_enable_flag == 2
#endif
        )
#else
    if (ctx->param.use_ibc_flag || ctx->param.fimc_enable_flag == 1)
#endif
#else
    if (ctx->param.use_ibc_flag)
#endif
    {
        ctx->ibc_hash_handle = create_enc_IBC(ctx->info.pic_width, ctx->info.pic_height);
    }
#endif

    /*  allocate CU data map*/
    if(ctx->map_cu_data == NULL)
    {
        size = sizeof(ENC_CU_DATA) * ctx->info.f_lcu;
        ctx->map_cu_data = (ENC_CU_DATA*) com_malloc_fast(size);
        com_assert_gv(ctx->map_cu_data, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map_cu_data, 0, size);
        for(i = 0 ; i < (int)ctx->info.f_lcu; i++)
        {
            enc_create_cu_data(ctx->map_cu_data + i, ctx->info.log2_max_cuwh - MIN_CU_LOG2, ctx->info.log2_max_cuwh - MIN_CU_LOG2
#if USE_SP
                , ctx->param.sp_enable_flag, ctx->param.evs_ubvs_enable_flag
#endif
            );
        }
    }
    /* allocate maps */
    if(ctx->map.map_scu == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_scu = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_scu, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_scu, 0, size);
    }
    if (ctx->map.map_split == NULL)
    {
        size = sizeof(s8) * ctx->info.f_lcu * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU;
        ctx->map.map_split = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_split, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_split, 0, size);
    }
    if(ctx->map.map_ipm == NULL)
    {
        size = sizeof(s8) * ctx->info.f_scu;
        ctx->map.map_ipm = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_ipm, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->map.map_ipm, -1, size);
    }
    if (ctx->map.map_patch_idx == NULL)
    {
        size = sizeof(s8)* ctx->info.f_scu;
        ctx->map.map_patch_idx = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_patch_idx, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->map.map_patch_idx, -1, size);
    }
#if TB_SPLIT_EXT
    if (ctx->map.map_pb_tb_part == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_pb_tb_part = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_pb_tb_part, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->map.map_pb_tb_part, 0xffffffff, size);
    }
#endif
    size = sizeof(s8) * ctx->info.f_scu;
    ctx->map.map_depth = com_malloc_fast(size);
    com_assert_gv(ctx->map.map_depth, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    com_mset(ctx->map.map_depth, -1, size);
    if (ctx->map.map_cu_mode == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_cu_mode = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_cu_mode, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_cu_mode, 0, size);
    }

    if (ctx->map.map_delta_qp == NULL)
    {
        size = sizeof(s8) * ctx->info.f_lcu;
        ctx->map.map_delta_qp = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_delta_qp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_delta_qp, 0, size);
    }

#if USE_SP
    if (ctx->map.map_usp == NULL)
    {
        size = sizeof(u8) * ctx->info.f_scu;
        ctx->map.map_usp = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_usp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_usp, 0, size);
    }
#endif

#if PATCH
#if USE_SP
    /* alloc USP map temp */
    if (ctx->map.map_usp_temp == NULL)
    {
        size = sizeof(u8) * ctx->info.f_scu;
        ctx->map.map_usp_temp = (u8*)com_malloc(size);
        com_assert_gv(ctx->map.map_usp_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_usp_temp, 0, size);
    }
#endif
    /* alloc SCU map temp */
    if (ctx->map.map_scu_temp == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_scu_temp = (u32*)com_malloc(size);
        com_assert_gv(ctx->map.map_scu_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_scu_temp, 0, size);
    }
    /* alloc cu mode SCU map temp */
    if (ctx->map.map_cu_mode_temp == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_cu_mode_temp = (u32*)com_malloc(size);
        com_assert_gv(ctx->map.map_cu_mode_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_cu_mode_temp, 0, size);
    }

    /* alloc mv map temp */
    if (ctx->map.map_mv_temp == NULL)
    {
        size = sizeof(s16) * ctx->info.f_scu * REFP_NUM * MV_D;
        ctx->map.map_mv_temp = com_malloc(size);
        com_assert_gv(ctx->map.map_mv_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_mv_temp, 0, size);
    }
    /* alloc refi map temp */
    if (ctx->map.map_refi_temp == NULL)
    {
        size = sizeof(s8) * ctx->info.f_scu * REFP_NUM;
        ctx->map.map_refi_temp = com_malloc(size);
        com_assert_gv(ctx->map.map_refi_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_refi_temp, -1, size);
    }
#endif

    ctx->pa.width        = ctx->info.pic_width;
    ctx->pa.height        = ctx->info.pic_height;
    ctx->pa.pad_l    = PIC_PAD_SIZE_L;
    ctx->pa.pad_c    = PIC_PAD_SIZE_C;
    ctx->pic_cnt     = 0;
    ctx->pic_icnt    = -1;
    ctx->dtr         = 0;
    ctx->ptr         = 0;
    ret = com_picman_init(&ctx->rpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, &ctx->pa);
    com_assert_g(COM_SUCCEEDED(ret), ERR);
    ctx->pico_max_cnt = 1 + (ctx->param.max_b_frames << 1) ;
    ctx->frm_rnum = ctx->param.max_b_frames;
    ctx->info.bit_depth_internal = ctx->param.bit_depth_internal;
    ctx->info.bit_depth_input = ctx->param.bit_depth_input;
    ctx->info.qp_offset_bit_depth = (8 * (ctx->info.bit_depth_internal - 8));

    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        ctx->pico_buf[i] = (ENC_PICO*)com_malloc(sizeof(ENC_PICO));
        com_assert_gv(ctx->pico_buf[i], ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->pico_buf[i], 0, sizeof(ENC_PICO));
    }

    create_edge_filter_avs2(ctx->info.pic_width, ctx->info.pic_height, ctx->edge_filter);

    com_malloc_3d_sao_stat_data(&(ctx->sao_stat_data),
                              ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                          ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C, NUM_SAO_NEW_TYPES);
    com_malloc_2d_sao_param(&(ctx->sao_blk_params),
                               ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                           ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_malloc_2d_sao_param(&ctx->rec_sao_blk_params,
                               ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                           ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);

#if ESAO
#if ESAO_ENH
    com_malloc_3d_esao_stat_data(&(ctx->esao_luma_data), ctx->info.f_lcu, ESAO_LABEL_NUM_Y, ESAO_LUMA_TYPES);
    com_malloc_3d_esao_stat_data(&(ctx->esao_chroma_data), ctx->info.f_lcu, ESAO_LABEL_NUM_MAX, N_C - 1);
    for (int comp = 0; comp < N_C; comp++)
    {
        ctx->best_esao_param[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
        ctx->temp_esao_param[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
        ctx->init_esao_param[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    }
#else
    com_malloc_2d_esao_stat_data(&(ctx->esao_luma_data), ESAO_LABEL_NUM_Y, ESAO_LUMA_TYPES);
    com_malloc_2d_esao_stat_data(&(ctx->esao_chroma_data), ESAO_LABEL_NUM_MAX, 2);
#endif
#if ESAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_esao_params == NULL)
    {
        ctx->info.pic_header.pic_esao_params = (ESAO_BLK_PARAM*)malloc(N_C * sizeof(ESAO_BLK_PARAM));
    }
    ctx->info.pic_header.pic_esao_params[Y_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->info.pic_header.pic_esao_params[V_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->info.pic_header.pic_esao_params[U_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#else
    ctx->pic_esao_params[Y_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->pic_esao_params[V_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->pic_esao_params[U_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#endif
#endif
#if CCSAO
#if CCSAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_ccsao_params==NULL)
    {
        ctx->info.pic_header.pic_ccsao_params = (CCSAO_BLK_PARAM*)malloc((N_C - 1) * sizeof(CCSAO_BLK_PARAM));
    }
#endif
    for (int comp = U_C-1; comp < N_C-1; comp++)
    {
#if CCSAO_ENHANCEMENT
        com_malloc_4d_ccsao_statdata(&(ctx->ccsao_chroma_data[comp]), ctx->info.f_lcu, CCSAO_BAND_NUM, CCSAO_BAND_NUM_C, CCSAO_TYPE_NUM);
#else
        com_malloc_3d_ccsao_statdata(&(ctx->ccsao_chroma_data[comp]), ctx->info.f_lcu, CCSAO_BAND_NUM, CCSAO_TYPE_NUM);
#endif
#if ECCSAO
        com_malloc_4d_ccsao_statdata(&(ctx->ccsao_edge_chroma_data[comp]), ctx->info.f_lcu, CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C, CCSAO_QUAN_NUM, CCSAO_EDGE_TYPE);
#endif
#if CCSAO_PH_SYNTAX
        ctx->info.pic_header.pic_ccsao_params[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#endif
        ctx->pic_ccsao_params[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
        ctx->best_ccsao_param[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
        ctx->temp_ccsao_param[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#if CCSAO_ENHANCEMENT
        ctx->init_ccsao_param[comp].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#endif
    }
#endif
#if !ALF_SHAPE
    create_alf_global_buffers(ctx);
    ctx->info.pic_header.alf_picture_param = ctx->enc_alf->alf_picture_param;
    ctx->info.pic_header.pic_alf_on = ctx->pic_alf_on;
#endif

#if NN_FILTER
    for (int comp = 0; comp < N_C; comp++)
    {
        if (ctx->info.pic_header.nnlf_lcu_enable_flag[comp] == NULL)
        {
            ctx->info.pic_header.nnlf_lcu_enable_flag[comp] = (u8*)com_malloc(ctx->info.f_scu * sizeof(u8));
        }
        if (ctx->info.pic_header.nnlf_lcu_set_index[comp] == NULL)
        {
            ctx->info.pic_header.nnlf_lcu_set_index[comp] = (u8*)com_malloc(ctx->info.f_scu * sizeof(u8));
        }
    }
#endif
#if ALO
    if (ctx->param.alo_enable_type == 1)
    {
        if (ctx->param.pic_width <= ctx->param.ctu_size && ctx->param.pic_height <= ctx->param.ctu_size)
        {
            ctx->param.alo_enable_type = 0;
        }
        else
        {
            ctx->pEncALO = createEncALO(ctx);
        }
    }
#endif

    return COM_OK;
ERR:
    for (i = 0; i < (int)ctx->info.f_lcu; i++)
    {
        enc_delete_cu_data(ctx->map_cu_data + i
#if USE_SP
            , ctx->param.sp_enable_flag, ctx->param.evs_ubvs_enable_flag
#endif
        );
    }
    com_mfree_fast(ctx->map_cu_data);
    com_mfree_fast(ctx->map.map_ipm);
    com_mfree_fast(ctx->map.map_patch_idx);
    com_mfree_fast(ctx->map.map_depth);
#if TB_SPLIT_EXT
    com_mfree_fast(ctx->map.map_pb_tb_part);
#endif
    com_mfree_fast(ctx->map.map_cu_mode);
    com_mfree_fast(ctx->map.map_delta_qp);
#if USE_SP
    com_mfree_fast(ctx->map.map_usp);
#endif
    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        com_mfree_fast(ctx->pico_buf[i]);
    }
    delete_edge_filter_avs2(ctx->edge_filter, pic_height);

    if (ctx->pic_sao != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_sao);
    }
    com_free_3d_sao_stat_data(&ctx->sao_stat_data,
                            ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_free_2d_sao_param(&ctx->sao_blk_params,
                             ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                         ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
    com_free_2d_sao_param(&ctx->rec_sao_blk_params,
                             ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                         ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
#if ESAO
    if (ctx->pic_esao != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_esao);
    }
#if ESAO_ENH
    com_free_3d_esao_stat_data(&ctx->esao_luma_data, ctx->info.f_lcu, ESAO_LABEL_NUM_Y);
    com_free_3d_esao_stat_data(&ctx->esao_chroma_data, ctx->info.f_lcu, ESAO_LABEL_NUM_MAX);
    for (int comp = 0; comp < N_C; comp++)
    {
        com_mfree(ctx->best_esao_param[comp].lcu_flag);
        com_mfree(ctx->temp_esao_param[comp].lcu_flag);
        com_mfree(ctx->init_esao_param[comp].lcu_flag);
    }
#else
    com_free_2d_esao_stat_data(&ctx->esao_luma_data, ESAO_LABEL_NUM_Y);
    com_free_2d_esao_stat_data(&ctx->esao_chroma_data, ESAO_LABEL_NUM_MAX);
#endif
#if ESAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_esao_params)
    {
        com_mfree(ctx->info.pic_header.pic_esao_params[Y_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params[U_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params[V_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params);
        ctx->info.pic_header.pic_esao_params = NULL;
    }
#else
    com_mfree(ctx->pic_esao_params[Y_C].lcu_flag);
    com_mfree(ctx->pic_esao_params[U_C].lcu_flag);
    com_mfree(ctx->pic_esao_params[V_C].lcu_flag);
#endif 
#endif 
#if CCSAO
#if CCSAO_ENHANCEMENT
    if (ctx->pic_ccsao[0] != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_ccsao[0]);
    }
    if (ctx->pic_ccsao[1] != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_ccsao[1]);
    }
#else
    if (ctx->pic_ccsao != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_ccsao);
    }
#endif
    for (int comp = U_C-1; comp < N_C-1; comp++)
    {
#if CCSAO_ENHANCEMENT
        com_free_4d_ccsao_statdata(&ctx->ccsao_chroma_data[comp], ctx->info.f_lcu, CCSAO_BAND_NUM, CCSAO_BAND_NUM_C);
#else
        com_free_3d_ccsao_statdata(&ctx->ccsao_chroma_data[comp], ctx->info.f_lcu, CCSAO_BAND_NUM);
#endif
#if ECCSAO
        com_free_4d_ccsao_statdata(&ctx->ccsao_edge_chroma_data[comp], ctx->info.f_lcu, CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C, CCSAO_QUAN_NUM);
#endif
        com_mfree(ctx->pic_ccsao_params[comp].lcu_flag);
        com_mfree(ctx->best_ccsao_param[comp].lcu_flag);
        com_mfree(ctx->temp_ccsao_param[comp].lcu_flag);
#if CCSAO_ENHANCEMENT
        com_mfree(ctx->init_ccsao_param[comp].lcu_flag);
#endif
    }
#if CCSAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_ccsao_params)
    {
        com_mfree(ctx->info.pic_header.pic_ccsao_params[U_C-1].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_ccsao_params[V_C-1].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_ccsao_params);
        ctx->info.pic_header.pic_ccsao_params = NULL;
    }
#endif
#endif
#if ALF_SHAPE
    if (ctx->info.sqh.adaptive_leveling_filter_enable_flag)
    {
#endif
        destroy_alf_global_buffers(ctx, ctx->info.log2_max_cuwh);
        ctx->info.pic_header.alf_picture_param = NULL;
        ctx->info.pic_header.pic_alf_on = NULL;
#if ALF_SHAPE
    }
#endif

#if NN_FILTER
    for (int comp = 0; comp < N_C; comp++)
    {
        com_mfree(ctx->info.pic_header.nnlf_lcu_enable_flag[comp]);
        com_mfree(ctx->info.pic_header.nnlf_lcu_set_index[comp]);
        ctx->info.pic_header.nnlf_lcu_enable_flag[comp] = NULL;
        ctx->info.pic_header.nnlf_lcu_set_index[comp] = NULL;
    }
#endif

    if(core)
    {
        core_free(core);
    }
    return ret;
}

#if AWP || SAWP
void com_free_4d_Buf(pel ****array4D, int wNum, int hNum, int Num)
{
    int i, j, k;
    if (array4D)
    {
        for (i = 0; i < wNum; i++)
        {
            if (array4D[i])
            {
                for (j = 0; j < hNum; j++)
                {
                    if (array4D[i][j])
                    {
                        for (k = 0; k < Num; k++)
                        {
                            free(array4D[i][j][k]);
                            array4D[i][j][k] = NULL;
                        }
                        free(array4D[i][j]);
                        array4D[i][j] = NULL;
                    }
                }
                free(array4D[i]);
                array4D[i] = NULL;
            }
        }
        free(array4D);
        array4D = NULL;
    }
}

void com_free_3d_Buf(pel ***array3D, int candNum, int compNum)
{
    int i, j;
    if (array3D)
    {
        for (i = 0; i < candNum; i++)
        {
            if (array3D[i])
            {
                for (j = 0; j < compNum; j++)
                {
                    if (array3D[i][j])
                    {
                        free(array3D[i][j]);
                        array3D[i][j] = NULL;
                    }
                }
                free(array3D[i]);
                array3D[i] = NULL;
            }
        }
        free(array3D);
        array3D = NULL;
    }
}

void com_free_3d_Buf_BOOL(BOOL ***array3D, int wNum, int hNum)
{
    int i, j;
    if (array3D)
    {
        for (i = 0; i < wNum; i++)
        {
            if (array3D[i])
            {
                for (j = 0; j < hNum; j++)
                {
                    if (array3D[i][j])
                    {
                        free(array3D[i][j]);
                        array3D[i][j] = NULL;
                    }
                }
                free(array3D[i]);
                array3D[i] = NULL;
            }
        }
        free(array3D);
        array3D = NULL;
    }
}

void enc_delete_awp_bufs(ENC_CTX * ctx)
{
    int wNum   = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    int hNum   = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    int awpNum = AWP_MODE_NUM;
    com_free_4d_Buf     (ctx->awp_weight0,     wNum, hNum, awpNum);
    com_free_4d_Buf     (ctx->awp_weight1,     wNum, hNum, awpNum);

    com_free_4d_Buf     (ctx->awp_weight0_scc,  wNum, hNum, awpNum);
    com_free_4d_Buf     (ctx->awp_weight1_scc,  wNum, hNum, awpNum);

    com_free_4d_Buf     (ctx->awp_bin_weight0, wNum, hNum, awpNum);
    com_free_4d_Buf     (ctx->awp_bin_weight1, wNum, hNum, awpNum);
#if FIX_372
    com_free_4d_Buf     (ctx->awp_weight0_bawp,  wNum, hNum, awpNum);
    com_free_4d_Buf     (ctx->awp_weight1_bawp,  wNum, hNum, awpNum);

    com_free_4d_Buf     (ctx->awp_bin_weight0_bawp, wNum, hNum, awpNum);
    com_free_4d_Buf     (ctx->awp_bin_weight1_bawp, wNum, hNum, awpNum);
#endif
    com_free_3d_Buf_BOOL(ctx->awp_larger_area,    wNum, hNum        );
    return;
}

void com_malloc_4d_Buf(pel *****array4D, int wNum, int hNum, int Num, int sizeNum)
{
    int i, j, k;
    if (((*array4D) = (pel ****)calloc(wNum, sizeof(pel ***))) == NULL)
    {
        printf("MALLOC FAILED: get_mem4DawpWeight0: array4D");
        assert(0);
        exit(-1);
    }

    for (i = 0; i < wNum; i++)
    {
        if ((*(*array4D + i) = (pel ***)calloc(hNum, sizeof(pel **))) == NULL)
        {
            printf("MALLOC FAILED: get_mem3DawpWeight0: array3D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < hNum; j++)
        {
            if ((*(*(*array4D + i) + j) = (pel **)calloc(Num, sizeof(pel *))) == NULL)
            {
                printf("MALLOC FAILED: get_mem2DawpWeight0: array2D");
                assert(0);
                exit(-1);
            }
            for (k = 0; k < Num; k++)
            {
                if ((*(*(*(*array4D + i) + j) + k) = (pel *)calloc(sizeNum, sizeof(pel))) == NULL)
                {
                    printf("MALLOC FAILED: get_mem1DawpWeight0: arrayD");
                    assert(0);
                    exit(-1);
                }
            }
        }
    }
    return;
}

void com_malloc_3d_Buf(pel ****array3D, int candNum, int compNum, int sizeNum)
{
    int i, j;
    if (((*array3D) = (pel ***)calloc(candNum, sizeof(pel **))) == NULL)
    {
        printf("MALLOC FAILED: get_mem3DawpWeight0: array3D");
        assert(0);
        exit(-1);
    }

    for (i = 0; i < candNum; i++)
    {
        if ((*(*array3D + i) = (pel **)calloc(compNum, sizeof(pel *))) == NULL)
        {
            printf("MALLOC FAILED: get_mem2DawpWeight0: array2D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < compNum; j++)
        {
            if ((*(*(*array3D + i) + j) = (pel *)calloc(sizeNum, sizeof(pel))) == NULL)
            {
                printf("MALLOC FAILED: get_mem1DawpWeight0: arrayD");
                assert(0);
                exit(-1);
            }
        }
    }
    return;
}

void com_malloc_3d_Buf_BOOL(BOOL ****array3D, int wNum, int hNum, int Num)
{
    int i, j;
    if (((*array3D) = (BOOL ***)calloc(wNum, sizeof(BOOL **))) == NULL)
    {
        printf("MALLOC FAILED: get_mem3DawpWeight0: array3D");
        assert(0);
        exit(-1);
    }

    for (i = 0; i < wNum; i++)
    {
        if ((*(*array3D + i) = (BOOL **)calloc(hNum, sizeof(BOOL *))) == NULL)
        {
            printf("MALLOC FAILED: get_mem2DawpWeight0: array2D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < hNum; j++)
        {
            if ((*(*(*array3D + i) + j) = (BOOL *)calloc(Num, sizeof(BOOL))) == NULL)
            {
                printf("MALLOC FAILED: get_mem1DawpWeight0: arrayD");
                assert(0);
                exit(-1);
            }
        }
    }
    return;
}

void enc_init_awp_bufs(ENC_CTX *ctx)
{
    int wNum    = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    int hNum    = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    int awpNum  = AWP_MODE_NUM;
    int sizeNum = MAX_AWP_SIZE * MAX_AWP_SIZE;
    com_malloc_4d_Buf     (&ctx->awp_weight0,     wNum, hNum, awpNum, sizeNum);
    com_malloc_4d_Buf     (&ctx->awp_weight1,     wNum, hNum, awpNum, sizeNum);

    com_malloc_4d_Buf     (&ctx->awp_weight0_scc,  wNum, hNum, awpNum, sizeNum);
    com_malloc_4d_Buf     (&ctx->awp_weight1_scc,  wNum, hNum, awpNum, sizeNum);

    com_malloc_4d_Buf     (&ctx->awp_bin_weight0, wNum, hNum, awpNum, sizeNum);
    com_malloc_4d_Buf     (&ctx->awp_bin_weight1, wNum, hNum, awpNum, sizeNum);
#if FIX_372
    com_malloc_4d_Buf     (&ctx->awp_weight0_bawp,  wNum, hNum, awpNum, sizeNum);
    com_malloc_4d_Buf     (&ctx->awp_weight1_bawp,  wNum, hNum, awpNum, sizeNum);

    com_malloc_4d_Buf     (&ctx->awp_bin_weight0_bawp, wNum, hNum, awpNum, sizeNum);
    com_malloc_4d_Buf     (&ctx->awp_bin_weight1_bawp, wNum, hNum, awpNum, sizeNum);
#endif
    com_malloc_3d_Buf_BOOL(&ctx->awp_larger_area,    wNum, hNum, awpNum         );
    return;
}
#endif

void enc_flush(ENC_CTX * ctx)
{
    int i;
    com_assert(ctx);
    com_mfree_fast(ctx->map.map_scu);
    com_mfree_fast(ctx->map.map_split);
    for (i = 0; i < (int)ctx->info.f_lcu; i++)
    {
        enc_delete_cu_data(ctx->map_cu_data + i
#if USE_SP
            , ctx->param.sp_enable_flag, ctx->param.evs_ubvs_enable_flag
#endif
        );
    }
    com_mfree_fast(ctx->map_cu_data);
    com_mfree_fast(ctx->map.map_ipm);
    com_mfree_fast(ctx->map.map_patch_idx);
#if TB_SPLIT_EXT
    com_mfree_fast(ctx->map.map_pb_tb_part);
#endif
    com_mfree_fast(ctx->map.map_depth);
    com_mfree_fast(ctx->map.map_cu_mode);
    com_mfree_fast(ctx->map.map_delta_qp);
#if USE_SP
    com_mfree_fast(ctx->map.map_usp);
#endif
#if RDO_DBK
    com_picbuf_free(ctx->pic_dbk);
#endif
#if USE_IBC
    if (ctx->param.use_ibc_flag)
    {
        com_picbuf_free(ctx->ibc_unfiltered_rec_pic);
    }
#endif
#if USE_SP
    if (ctx->param.sp_enable_flag|| ctx->param.evs_ubvs_enable_flag)
    {
        release_sp_instance(ctx->sp_encoder);
    }
#endif
    com_picbuf_free(ctx->pic_alf_Org);
    com_picbuf_free(ctx->pic_alf_Rec);
    com_picman_deinit(&ctx->rpm);
#if LIBVC_ON
    ctx->rpm.libvc_data = NULL;
#endif
    core_free(ctx->core);
    for(i = 0; i < ctx->pico_max_cnt; i++)
    {
        com_mfree_fast(ctx->pico_buf[i]);
    }
    for(i = 0; i < ENC_MAX_INBUF_CNT; i++)
    {
        if(ctx->inbuf[i]) ctx->inbuf[i]->release(ctx->inbuf[i]);
    }

    delete_edge_filter_avs2(ctx->edge_filter, ctx->info.pic_height);

    if (ctx->pic_sao != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_sao);
    }
    com_free_3d_sao_stat_data(&ctx->sao_stat_data,
                            ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_free_2d_sao_param(&ctx->sao_blk_params,
                             ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                         ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
    com_free_2d_sao_param(&ctx->rec_sao_blk_params,
                             ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                         ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
#if ESAO
    if (ctx->pic_esao != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_esao);
    }
#if ESAO_ENH
    com_free_3d_esao_stat_data(&ctx->esao_luma_data, ctx->info.f_lcu, ESAO_LABEL_NUM_Y);
    com_free_3d_esao_stat_data(&ctx->esao_chroma_data, ctx->info.f_lcu, ESAO_LABEL_NUM_MAX);
    for (int comp = 0; comp < N_C; comp++)
    {
        com_mfree(ctx->best_esao_param[comp].lcu_flag);
        com_mfree(ctx->temp_esao_param[comp].lcu_flag);
        com_mfree(ctx->init_esao_param[comp].lcu_flag);
    }
#else
    com_free_2d_esao_stat_data(&ctx->esao_luma_data, ESAO_LABEL_NUM_Y);
    com_free_2d_esao_stat_data(&ctx->esao_chroma_data, ESAO_LABEL_NUM_MAX);
#endif
#if ESAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_esao_params)
    {
        com_mfree(ctx->info.pic_header.pic_esao_params[Y_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params[U_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params[V_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params);
        ctx->info.pic_header.pic_esao_params = NULL;
    }

#else
    com_mfree(ctx->pic_esao_params[Y_C].lcu_flag);
    com_mfree(ctx->pic_esao_params[U_C].lcu_flag);
    com_mfree(ctx->pic_esao_params[V_C].lcu_flag);
#endif
#endif
#if CCSAO
#if CCSAO_ENHANCEMENT
    if (ctx->pic_ccsao[0] != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_ccsao[0]);
    }
    if (ctx->pic_ccsao[1] != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_ccsao[1]);
    }
#else
    if (ctx->pic_ccsao != NULL)
    {
        com_pic_free(&ctx->pa, ctx->pic_ccsao);
    }
#endif
    for (int comp = U_C-1; comp < N_C-1; comp++)
    {
#if CCSAO_ENHANCEMENT
        com_free_4d_ccsao_statdata(&ctx->ccsao_chroma_data[comp], ctx->info.f_lcu, CCSAO_BAND_NUM, CCSAO_BAND_NUM_C);
#else
        com_free_3d_ccsao_statdata(&ctx->ccsao_chroma_data[comp], ctx->info.f_lcu, CCSAO_BAND_NUM);
#endif
#if ECCSAO
        com_free_4d_ccsao_statdata(&ctx->ccsao_edge_chroma_data[comp], ctx->info.f_lcu, CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C, CCSAO_QUAN_NUM);
#endif
#if CCSAO_PH_SYNTAX
        com_mfree(ctx->info.pic_header.pic_ccsao_params[comp].lcu_flag);
#endif
        com_mfree(ctx->pic_ccsao_params[comp].lcu_flag);
        com_mfree(ctx->best_ccsao_param[comp].lcu_flag);
        com_mfree(ctx->temp_ccsao_param[comp].lcu_flag);
#if CCSAO_ENHANCEMENT
        com_mfree(ctx->init_ccsao_param[comp].lcu_flag);
#endif
    }
#endif
    if (ctx->info.sqh.adaptive_leveling_filter_enable_flag)
    {
        destroy_alf_global_buffers(ctx, ctx->info.log2_max_cuwh);
    }
    ctx->info.pic_header.alf_picture_param = NULL;
    ctx->info.pic_header.pic_alf_on = NULL;

#if NN_FILTER
    for (int comp = 0; comp < N_C; comp++)
    {
        com_mfree(ctx->info.pic_header.nnlf_lcu_enable_flag[comp]);
        com_mfree(ctx->info.pic_header.nnlf_lcu_set_index[comp]);
        ctx->info.pic_header.nnlf_lcu_enable_flag[comp] = NULL;
        ctx->info.pic_header.nnlf_lcu_set_index[comp] = NULL;
    }
#endif

#if PATCH
#if USE_SP
    if (ctx->map.map_usp_temp)
    {
        com_mfree(ctx->map.map_usp_temp);
        ctx->map.map_usp_temp = NULL;
    }
#endif
    if (ctx->map.map_scu_temp)
    {
        com_mfree(ctx->map.map_scu_temp);
        ctx->map.map_scu_temp = NULL;
    }
    if (ctx->map.map_cu_mode_temp)
    {
        com_mfree(ctx->map.map_cu_mode_temp);
        ctx->map.map_cu_mode_temp = NULL;
    }
    if (ctx->map.map_mv_temp)
    {
        com_mfree(ctx->map.map_mv_temp);
        ctx->map.map_mv_temp = NULL;
    }
    if (ctx->map.map_refi_temp)
    {
        com_mfree(ctx->map.map_refi_temp);
        ctx->map.map_refi_temp = NULL;
    }
#endif
}

int enc_deblock_avs2(ENC_CTX * ctx, COM_PIC * pic
#if DBR
    , COM_PIC * pic_org
#endif
)
{
#if DBR
    int scale_lambda = (ctx->info.bit_depth_internal == 10) ? ctx->info.qp_offset_bit_depth : 1;
    double    lambda = ctx->lambda[0] * scale_lambda;
#endif
    clear_edge_filter_avs2(0, 0, pic->width_luma, pic->height_luma, ctx->edge_filter);
    set_edge_filter_avs2(&ctx->info, &ctx->map, pic, ctx->edge_filter);

#if DBR
#if SH_UNIFY
    if (ctx->info.sqh.dbr_enable_flag && ctx->param.dbr_enable_flag) // control by cfg and profile
#else
    if (ctx->info.sqh.dbr_enable_flag)
#endif
    {
        enc_deblock_frame(ctx, &ctx->info, &ctx->map, pic, pic_org, ctx->refp, ctx->edge_filter, lambda);
    }
    else
    {
        deblock_frame_avs2(&ctx->info, &ctx->map, pic, ctx->refp, ctx->edge_filter);
    }
#else
    deblock_frame_avs2(&ctx->info, &ctx->map, pic, ctx->refp, ctx->edge_filter);
#endif

    return COM_OK;
}

#if DBR
//deblock one frame in the encoder
void enc_deblock_frame(ENC_CTX * ctx, COM_INFO *info, COM_MAP *map, COM_PIC * pic, COM_PIC * pic_org, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter, double lambda)
{
    int mb_x, mb_y;
    int md1 = 0, md2 = 0, min_index = 0, mnum = 0;
    int d1  = 0, d2  = 0;
    double cost = 0, mcost = 0, dist = 0;
#if EDBR
    int md1_fs0 = 0, md2_fs0 = 0;
    int d1_fs0  = 0, d2_fs0  = 0;
    double dist_fs0 = 0;
#endif
    int cur_dbr_idx  = 0, dbr_poc_base = 8;
    int cur_pic_poc  = ctx->param.gop_size == 1 ? ctx->pic_cnt : ctx->ptr;

    DBR_PARAM *dbr_pic_param = &(info->pic_header.ph_dbr_param);
    int *offsets_h = dbr_pic_param->horizontal_offsets;
    int *offsets_v = dbr_pic_param->vertical_offsets;

    for (int k = 1; k <= 2; k++)
    {
#if EDBR
        memset(offsets_v, 0, 10 * sizeof(int));
#else
        memset(offsets_v, 0, 6 * sizeof(int));
#endif

        for (mb_y = 0; mb_y < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
        {
            for (mb_x = 0; mb_x < (pic->width_luma >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
            {
                deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 0, pic_org, k, dbr_pic_param
#if RDO_DBK_LUMA_ONLY
                    , 1
#endif
                );
            }
        }

        d1 = (offsets_v[2] > 0) ? (offsets_v[0] + (offsets_v[2] >> 1) * (offsets_v[0] < 0 ? -1 : 1)) / offsets_v[2] : 0;
        d2 = (offsets_v[3] > 0) ? (offsets_v[1] + (offsets_v[3] >> 1) * (offsets_v[1] < 0 ? -1 : 1)) / offsets_v[3] : 0;
        
        d1 = COM_CLIP3(-4, -1, d1);
        d2 = COM_CLIP3(1, 4, d2);

        dist = (double)(offsets_v[4] + offsets_v[5]) + ((double)(offsets_v[2]) * d1 * d1 - (double)(offsets_v[0]) * d1 * 2) + ((double)(offsets_v[3]) * d2 * d2 - (double)(offsets_v[1]) * d2 * 2);
       
#if EDBR
        if (dist > 0)
        {
            dist = 0;
            d1 = 0;
            d2 = 0;
        }
        if ((ctx->param.i_period > 1 && pic->height_luma <= 720 && !(info->pic_header.slice_type == SLICE_I && ctx->param.qp <= 27)) || (ctx->param.i_period == 0 && pic->height_luma <= 1080 && !(info->pic_header.slice_type == SLICE_I && ctx->param.qp >= 45)))
        {
            dist_fs0 = 0;
            d1_fs0 = 0;
            d2_fs0 = 0;
        }
        else
        {
            d1_fs0 = (offsets_v[2 + 6] > 0) ? (offsets_v[0 + 6] + (offsets_v[2 + 6] >> 1) * (offsets_v[0 + 6] < 0 ? -1 : 1)) / offsets_v[2 + 6] : 0;
            d2_fs0 = (offsets_v[3 + 6] > 0) ? (offsets_v[1 + 6] + (offsets_v[3 + 6] >> 1) * (offsets_v[1 + 6] < 0 ? -1 : 1)) / offsets_v[3 + 6] : 0;

            d1_fs0 = COM_CLIP3(-4, -1, d1_fs0);
            d2_fs0 = COM_CLIP3(1, 4, d2_fs0);

            dist_fs0 = ((double)(offsets_v[2 + 6]) * d1_fs0 * d1_fs0 - (double)(offsets_v[0 + 6]) * d1_fs0 * 2)
                + ((double)(offsets_v[3 + 6]) * d2_fs0 * d2_fs0 - (double)(offsets_v[1 + 6]) * d2_fs0 * 2);
            if (dist_fs0 > 0)
            {
                dist_fs0 = 0;
                d1_fs0 = 0;
                d2_fs0 = 0;
            }
            dist += dist_fs0;
        }
#endif

        cost = dist + lambda * 5.0;

        if (cost < mcost)
        {
            mcost     = cost;
            min_index = k;
            md1 = d1;
            md2 = d2;
            mnum = offsets_v[2] + offsets_v[3];
#if EDBR
            md1_fs0 = d1_fs0;
            md2_fs0 = d2_fs0;
#endif
        }
    }

    if ((mcost < 0.0) && (mnum > 8)) 
    {
        dbr_pic_param->thresh_vertical_index = min_index;
        dbr_pic_param->vertical_offsets[0]   = -md1;
        dbr_pic_param->vertical_offsets[1]   = md2;
#if EDBR
        dbr_pic_param->vertical_offsets[0 + 6] = -md1_fs0;
        dbr_pic_param->vertical_offsets[1 + 6] = md2_fs0;
        dbr_pic_param->dbr_fs0_vertical_enabled = (md1_fs0 != 0 || md2_fs0 != 0);
#endif
        dbr_pic_param->dbr_vertical_enabled = (md1 != 0 || md2 != 0);
       
    }
    else
    {
        dbr_pic_param->dbr_vertical_enabled = FALSE;
#if EDBR
        dbr_pic_param->dbr_fs0_vertical_enabled = FALSE;
#endif
    }

    for (mb_y = 0; mb_y < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
    {
        for (mb_x = 0; mb_x < (pic->width_luma >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
        {
            //vertical
            deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 0, NULL, 0, dbr_pic_param
#if RDO_DBK_LUMA_ONLY
                , 0
#endif
            );
        }
    }

    //initialization
    md1       = 0;
    md2       = 0;
#if EDBR
    md1_fs0   = 0;
    md2_fs0   = 0;
#endif
    min_index = 0;
    mnum      = 0;
    mcost     = 0;

    for (int k = 1; k <= 2; k++)
    {
#if EDBR
        memset(offsets_h, 0, 10 * sizeof(int));
#else
        memset(offsets_h, 0, 6 * sizeof(int));
#endif

        for (mb_y = 0; mb_y < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
        {
            for (mb_x = 0; mb_x < (pic->width_luma >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
            {
                deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 1, pic_org, k, dbr_pic_param
#if RDO_DBK_LUMA_ONLY
                    , 1
#endif
                );
            }
        }

        d1 = (offsets_h[2] > 0) ? (offsets_h[0] + (offsets_h[2] >> 1) * (offsets_h[0] < 0 ? -1 : 1)) / offsets_h[2] : 0;
        d2 = (offsets_h[3] > 0) ? (offsets_h[1] + (offsets_h[3] >> 1) * (offsets_h[1] < 0 ? -1 : 1)) / offsets_h[3] : 0;
        d1 = COM_CLIP3(-4, -1, d1);
        d2 = COM_CLIP3(1, 4, d2);

        dist = (double)(offsets_h[4] + offsets_h[5])
            + ((double)(offsets_h[2]) * d1 * d1 - (double)(offsets_h[0]) * d1 * 2)
            + ((double)(offsets_h[3]) * d2 * d2 - (double)(offsets_h[1]) * d2 * 2);

#if EDBR
        if (dist > 0)
        {
            dist = 0;
            d1 = 0;
            d2 = 0;
        }
        if ((ctx->param.i_period > 1 && pic->height_luma <= 720 && !(info->pic_header.slice_type == SLICE_I && ctx->param.qp <= 27)) || (ctx->param.i_period == 0 && pic->height_luma <= 1080 && !(info->pic_header.slice_type == SLICE_I && ctx->param.qp >= 45)))
        {
            dist_fs0 = 0;
            d1_fs0 = 0;
            d2_fs0 = 0;
        }
        else
        {
            d1_fs0 = (offsets_h[2 + 6] > 0) ? (offsets_h[0 + 6] + (offsets_h[2 + 6] >> 1) * (offsets_h[0 + 6] < 0 ? -1 : 1)) / offsets_h[2 + 6] : 0;
            d2_fs0 = (offsets_h[3 + 6] > 0) ? (offsets_h[1 + 6] + (offsets_h[3 + 6] >> 1) * (offsets_h[1 + 6] < 0 ? -1 : 1)) / offsets_h[3 + 6] : 0;

            d1_fs0 = COM_CLIP3(-4, -1, d1_fs0);
            d2_fs0 = COM_CLIP3(1, 4, d2_fs0);

            dist_fs0 = ((double)(offsets_h[2 + 6]) * d1_fs0 * d1_fs0 - (double)(offsets_h[0 + 6]) * d1_fs0 * 2)
                + ((double)(offsets_h[3 + 6]) * d2_fs0 * d2_fs0 - (double)(offsets_h[1 + 6]) * d2_fs0 * 2);
            if (dist_fs0 > 0)
            {
                dist_fs0 = 0;
                d1_fs0 = 0;
                d2_fs0 = 0;
            }
            dist += dist_fs0;
        }
#endif

        cost = dist + lambda * 5.0;

        if (cost < mcost)
        {
            mcost     = cost;
            min_index = k;
            md1       = d1;
            md2       = d2;
#if EDBR
            md1_fs0   = d1_fs0;
            md2_fs0   = d2_fs0;
#endif
            mnum      = offsets_h[2] + offsets_h[3];
        }
    }

    if ((mcost < 0.0) && (mnum > 8))
    {
        dbr_pic_param->thresh_horizontal_index = min_index;
        dbr_pic_param->horizontal_offsets[0]   = -md1;
        dbr_pic_param->horizontal_offsets[1]   = md2;
#if EDBR
        dbr_pic_param->horizontal_offsets[0 + 6] = -md1_fs0;
        dbr_pic_param->horizontal_offsets[1 + 6] = md2_fs0;
        dbr_pic_param->dbr_fs0_horizontal_enabled = (md1_fs0 != 0 || md2_fs0 != 0);
#endif
        dbr_pic_param->dbr_horizontal_enabled  = (md1 != 0 || md2 != 0);

    }
    else
    {
        dbr_pic_param->dbr_horizontal_enabled = FALSE;
#if EDBR
        dbr_pic_param->dbr_fs0_horizontal_enabled = FALSE;
#endif
    }

    for (mb_y = 0; mb_y < (pic->height_luma >> LOOPFILTER_SIZE_IN_BIT); mb_y++)
    {
        for (mb_x = 0; mb_x < (pic->width_luma >> LOOPFILTER_SIZE_IN_BIT); mb_x++)
        {
            //horizontal
            deblock_mb_avs2(info, map, pic, refp, edge_filter, mb_y, mb_x, 1, NULL, 0, dbr_pic_param
#if RDO_DBK_LUMA_ONLY
                , 0
#endif
            );
        }
    }

    //Update the parameters buffer
    ctx->dbr_pic_poc[0] = cur_pic_poc;
    dbr_param_copy(&(ctx->dbr_pic_param[0]), &(info->pic_header.ph_dbr_param));
    if (2 == (cur_pic_poc % dbr_poc_base))
    {
        cur_dbr_idx = 1;
    }
    else if (4 == (cur_pic_poc % dbr_poc_base))
    {
        cur_dbr_idx = 2;
    }
    else if (6 == (cur_pic_poc % dbr_poc_base))
    {
        cur_dbr_idx = 3;
    }
    else if (0 == (cur_pic_poc % dbr_poc_base))
    {
        if (0 == (cur_pic_poc % (dbr_poc_base * 2)))
        {
            cur_dbr_idx = 5;
        }
        else
        {
            cur_dbr_idx = 4;
        }
    }
    if (cur_dbr_idx != 0)
    {
        ctx->dbr_pic_poc[cur_dbr_idx] = cur_pic_poc;
        dbr_param_copy(&(ctx->dbr_pic_param[cur_dbr_idx]), &(info->pic_header.ph_dbr_param));
    }
}
#endif

int enc_alf_avs2(ENC_CTX * ctx, COM_BSW *alf_bs_temp, COM_PIC * pic_rec, COM_PIC * pic_org)
{
    ALF_PARAM **final_alf_picture_param = ctx->enc_alf->alf_picture_param;
    int bit_depth = ctx->info.bit_depth_internal;
    int scale_lambda = (bit_depth == 10) ? ctx->info.qp_offset_bit_depth : 1;
    ctx->pic_alf_Rec->stride_luma = pic_rec->stride_luma;
    ctx->pic_alf_Rec->stride_chroma = pic_rec->stride_chroma;
    ctx->pic_alf_Org->stride_luma = pic_rec->stride_luma;
    ctx->pic_alf_Org->stride_chroma = pic_rec->stride_chroma;
    copy_frame_for_alf(ctx->pic_alf_Rec, pic_rec);
    copy_frame_for_alf(ctx->pic_alf_Org, pic_org);
    alf_process(ctx, alf_bs_temp, final_alf_picture_param, pic_rec, ctx->pic_alf_Org, ctx->pic_alf_Rec, ctx->lambda[0] * scale_lambda);
    for (int i = 0; i < N_C; i++)
    {
        ctx->pic_alf_on[i] = final_alf_picture_param[i]->alf_flag;
    }
    return COM_OK;
}

int enc_picbuf_get_inbuf(ENC_CTX * ctx, COM_IMGB ** imgb)
{
    int i, opt, align[COM_IMGB_MAX_PLANE], pad[COM_IMGB_MAX_PLANE];
    for(i=0; i<ENC_MAX_INBUF_CNT; i++)
    {
        if(ctx->inbuf[i] == NULL)
        {
            opt = COM_IMGB_OPT_NONE;
            /* set align value*/
            align[0] = MIN_CU_SIZE;
            align[1] = MIN_CU_SIZE >> 1;
            align[2] = MIN_CU_SIZE >> 1;
            /* no padding */
            pad[0] = 0;
            pad[1] = 0;
            pad[2] = 0;
            *imgb = com_imgb_create(ctx->param.pic_width, ctx->param.pic_height, COM_COLORSPACE_YUV420, pad, align);
            com_assert_rv(*imgb != NULL, COM_ERR_OUT_OF_MEMORY);
            ctx->inbuf[i] = *imgb;
            (*imgb)->addref(*imgb);
            return COM_OK;
        }
        else if(ctx->inbuf[i]->getref(ctx->inbuf[i]) == 1)
        {
            *imgb = ctx->inbuf[i];
            (*imgb)->addref(*imgb);
            return COM_OK;
        }
    }
    return COM_ERR_UNEXPECTED;
}

#if REPEAT_SEQ_HEADER
int init_seq_header( ENC_CTX * ctx, COM_BITB * bitb )
{
    COM_BSW * bs;
    COM_SQH * sqh;
    bs = &ctx->bs;
    sqh = &ctx->info.sqh;
    bs->pdata[1] = &ctx->sbac_enc; //necessary
    set_sqh( ctx, sqh );
#if ALF_SHAPE
    if(ctx->info.sqh.adaptive_leveling_filter_enable_flag)
    {
        create_alf_global_buffers(ctx); // if ALF is closed 
        ctx->info.pic_header.alf_picture_param = ctx->enc_alf->alf_picture_param;
        ctx->info.pic_header.pic_alf_on = ctx->pic_alf_on;
    }
#endif
    return COM_OK;
}
#else
int enc_seq_header(ENC_CTX * ctx, COM_BITB * bitb, ENC_STAT * stat)
{
    COM_BSW * bs;
    COM_SQH * sqh;
    com_assert_rv(bitb->addr && bitb->bsize > 0, COM_ERR_INVALID_ARGUMENT);
    bs     = &ctx->bs;
    sqh  = &ctx->info.sqh;
    /* bitsteam initialize for sequence */
    com_bsw_init(bs, bitb->addr, bitb->addr2, bitb->bsize, NULL);
    bs->pdata[1] = &ctx->sbac_enc;
    /* encode sequence header *************************************************/
    /* sequence header */
    set_sqh(ctx, sqh);
    com_assert_rv(enc_eco_sqh(ctx, bs, sqh)==COM_OK, COM_ERR_INVALID_ARGUMENT);
    /* de-init BSW */
    com_bsw_deinit(bs);
    /* set stat ***************************************************************/
    com_mset(stat, 0, sizeof(ENC_STAT));
    stat->write = COM_BSW_GET_WRITE_BYTE(bs);
    stat->ctype    = COM_CT_SQH;
    return COM_OK;
}
#endif

static void decide_normal_gop(ENC_CTX * ctx, u32 pic_imcnt)
{
    int i_period, gop_size,  pos;
    u32        pic_icnt_b;
    i_period  = ctx->param.i_period;
    gop_size  = ctx->param.gop_size;
    if(i_period == 0 && pic_imcnt == 0)
    {
        ctx->slice_type  = SLICE_I;
        ctx->slice_depth = FRM_DEPTH_0;
        ctx->poc         = pic_imcnt;
#if !RPL_GOP32
        ctx->slice_ref_flag = 1;
#endif
    }
    else if((i_period!= 0) && pic_imcnt % i_period  == 0)
    {
        ctx->slice_type  = SLICE_I;
        ctx->slice_depth = FRM_DEPTH_0;
        ctx->poc         = pic_imcnt;
#if !RPL_GOP32
        ctx->slice_ref_flag = 1;
#endif
    }
    else if(pic_imcnt % gop_size == 0)
    {
        ctx->slice_type  = SLICE_B;
#if !RPL_GOP32
        ctx->slice_ref_flag = 1;
#endif
        ctx->slice_depth = FRM_DEPTH_1;
        ctx->poc         = pic_imcnt;
#if !RPL_GOP32
        ctx->slice_ref_flag = 1;
#endif
    }
    else
    {
        ctx->slice_type  = SLICE_B;
        if( !ctx->param.disable_hgop )
        {
            pos              = (pic_imcnt % gop_size) - 1;
            ctx->slice_depth = tbl_slice_depth[gop_size >> 2][pos];
#if !RPL_GOP32
            ctx->ref_depth   = tbl_ref_depth[gop_size >> 2][pos];
#endif
            ctx->poc         = ((pic_imcnt / gop_size) * gop_size) +
                               tbl_poc_gop_offset[gop_size >> 2][pos];
#if !RPL_GOP32
            ctx->slice_ref_flag = tbl_slice_ref[gop_size >> 2][pos];
#endif
        }
        else
        {
            pos              = (pic_imcnt % gop_size) - 1;
            ctx->slice_depth = FRM_DEPTH_2;
#if !RPL_GOP32
            ctx->ref_depth   = FRM_DEPTH_1;
#endif
            ctx->poc         = ((pic_imcnt / gop_size) * gop_size) - gop_size + pos +1;
#if !RPL_GOP32
            ctx->slice_ref_flag = 0;
#endif
        }
        /* find current encoding picture's(B picture) pic_icnt */
        pic_icnt_b    =  ctx->poc;
        /* find pico again here */
        ctx->pico_idx = (u8)(pic_icnt_b % ctx->pico_max_cnt);
        ctx->pico = ctx->pico_buf[ctx->pico_idx];
        PIC_ORG(ctx) = &ctx->pico->pic;
    }
}

/* slice_type / slice_depth / poc / PIC_ORIG setting */
static void decide_slice_type(ENC_CTX * ctx)
{
    int pic_imcnt, pic_icnt;
    int i_period, gop_size;
    int force_cnt =  0;
    i_period  = ctx->param.i_period;
    gop_size  = ctx->param.gop_size;
    pic_icnt  = (ctx->pic_cnt + ctx->param.max_b_frames);
    pic_imcnt = pic_icnt;
    ctx->pico_idx = pic_icnt % ctx->pico_max_cnt;
    ctx->pico     = ctx->pico_buf[ctx->pico_idx];
    PIC_ORG(ctx) = &ctx->pico->pic;
    if(gop_size == 1)
    {
        pic_imcnt = (i_period > 0) ? pic_icnt % i_period : pic_icnt;
        if(pic_imcnt == 0) // all intra configuration
        {
            ctx->slice_type  = SLICE_I;
            ctx->slice_depth = FRM_DEPTH_0;
#if LIBVC_ON
            if (ctx->rpm.libvc_data->is_libpic_processing)
            {
                ctx->poc = pic_icnt;
            }
            else
#endif
            {
                ctx->poc = (i_period > 0) ? ctx->pic_cnt % i_period : ctx->pic_cnt;
            }
#if !RPL_GOP32
            ctx->slice_ref_flag = 1;
#endif
        }
        else
        {
            ctx->slice_type  = SLICE_B;
            if( !ctx->param.disable_hgop )
            {
                ctx->slice_depth = tbl_slice_depth_P[(pic_imcnt-1) % GOP_P];
            }
            else
            {
                ctx->slice_depth = FRM_DEPTH_1;
            }
            ctx->poc         = (i_period > 0) ? ctx->pic_cnt % i_period : ctx->pic_cnt;
#if !RPL_GOP32
            ctx->slice_ref_flag = 1;
#endif
        }
    }
    else /* include B Picture (gop_size = 2 or 4 or 8 or 16) */
    {
        if(pic_icnt == gop_size - 1) /* special case when sequence start */
        {
            ctx->slice_type  = SLICE_I;
            ctx->slice_depth = FRM_DEPTH_0;
            ctx->poc         = 0;
#if !RPL_GOP32
            ctx->slice_ref_flag = 1;
#endif
            /* flush the first IDR picture */
            PIC_ORG(ctx) = &ctx->pico_buf[0]->pic;
            ctx->pico     = ctx->pico_buf[0];
        }
        else if(ctx->force_slice)
        {
            for(force_cnt = ctx->force_ignored_cnt; force_cnt < gop_size; force_cnt++)
            {
                pic_icnt = (ctx->pic_cnt + ctx->param.max_b_frames + force_cnt);
                pic_imcnt = pic_icnt;
                decide_normal_gop(ctx, pic_imcnt);
                if(ctx->poc <= (int)ctx->pic_ticnt)
                {
                    break;
                }
            }
            ctx->force_ignored_cnt = force_cnt;
        }
        else /* normal GOP case */
        {
            decide_normal_gop(ctx, pic_imcnt);
        }
    }
    if( !ctx->param.disable_hgop )
    {
        ctx->temporal_id = ctx->slice_depth;
    }
    else
    {
        ctx->temporal_id = 0;
    }
    ctx->ptr = ctx->poc;
    ctx->dtr = ctx->ptr;
    ctx->info.pic_header.decode_order_index = ctx->pic_cnt;
}

#if EXTENSION_USER_DATA
#if TEMPORAL_SCALABILITY_EXTENSION
static int set_temporal_scalability_extension(ENC_CTX * ctx)
{
    COM_EXTENSION_DATA *extension_data = &(ctx->ctx_extension_data);
    if (extension_data->isTemporalInitial == 1) {
        return COM_OK;
    }
    extension_data->isTemporalInitial = 1;
    static int fps_code_tbl[14] = { -1, -1, 24, 25, -1, 30, 50, -1, 60, 100, 120, 200, 240, 300 };
    int i = 0;
    int fps = ctx->param.fps;
    int half_fps_code = -1;
    int fps_code = -1;
    for (i = 0; i < 14; i++)
    {
        if ((fps / 2) == fps_code_tbl[i])
        {
            half_fps_code = i;
        }
        if (fps == fps_code_tbl[i])
        {
            fps_code = i;
            break;
        }
    }
    /* set for GOP16 */
    COM_TEMPORAL_SCALABILITY *temporal_scalability = &(extension_data->extension_temporal_scalability);
    temporal_scalability->num_of_temporal_level = 6;
    int frame_rate_code[6] = {
        0xF,              // temporal id: 0
        0xF,              // temporal id: 1
        0xF,              // temporal id: 2
        0xF,              // temporal id: 3
        half_fps_code,    // temporal id: 4
        fps_code,         // temporal id: 5
    };
    for (i = 0; i < temporal_scalability->num_of_temporal_level; i++)
    {
        temporal_scalability->temporal_frame_rate[i] = frame_rate_code[i];
        temporal_scalability->temporal_bit_rate_lower[i] = 0;
        temporal_scalability->temporal_bit_rate_upper[i] = 0;
    }
    return COM_OK;
}
#endif
int enc_pic_extension_data_prepare(ENC_CTX * ctx)
{
#if TEMPORAL_SCALABILITY_EXTENSION
    /* initialize temporal scalability */
    int ret = set_temporal_scalability_extension(ctx);
#endif
    return COM_OK;
}
#endif

int enc_pic_prepare(ENC_CTX * ctx, COM_BITB * bitb)
{
    ENC_PARAM   * param;
    int             ret;
    int             size;
    com_assert_rv(PIC_ORG(ctx) != NULL, COM_ERR_UNEXPECTED);
    param = &ctx->param;
    PIC_REC(ctx) = com_picman_get_empty_pic(&ctx->rpm, &ret);
    com_assert_rv(PIC_REC(ctx) != NULL, ret);
    ctx->map.map_refi = PIC_REC(ctx)->map_refi;
    ctx->map.map_mv   = PIC_REC(ctx)->map_mv;
#if RDO_DBK
    if(ctx->pic_dbk == NULL)
    {
        ctx->pic_dbk = com_pic_alloc(&ctx->rpm.pa, &ret);
        com_assert_rv(ctx->pic_dbk != NULL, ret);
    }
#endif
#if USE_IBC
#if FIMC
#if ISTS
    if (ctx->param.use_ibc_flag || ctx->param.fimc_enable_flag == 1 || ctx->param.ists_enable_flag == 1
#if TS_INTER
        || ctx->param.ts_inter_enable_flag == 1
#endif
#if DBK_SCC
        || ctx->param.loop_filter_type_enable_flag == 1
#endif
#if AWP
        || ctx->param.awp_enable_flag == 2
#endif
        )
#else
    if (ctx->param.use_ibc_flag || ctx->param.fimc_enable_flag == 1)
#endif
#else
    if (ctx->param.use_ibc_flag)
#endif
    {
        if (ctx->ibc_unfiltered_rec_pic == NULL)
        {
            ctx->ibc_unfiltered_rec_pic = com_pic_alloc(&ctx->rpm.pa, &ret);
            com_assert_rv(ctx->ibc_unfiltered_rec_pic != NULL, ret);
        }
    }
#endif

    if (ctx->pic_sao == NULL)
    {
        ctx->pic_sao = com_pic_alloc(&ctx->rpm.pa, &ret);
        com_assert_rv(ctx->pic_sao != NULL, ret);
    }
#if ESAO
    if (ctx->pic_esao == NULL)
    {
        ctx->pic_esao = com_pic_alloc(&ctx->rpm.pa, &ret);
        com_assert_rv(ctx->pic_esao != NULL, ret);
    }
#endif
#if CCSAO
#if CCSAO_ENHANCEMENT
    if (ctx->pic_ccsao[0] == NULL)
    {
        ctx->pic_ccsao[0] = com_pic_alloc(&ctx->rpm.pa, &ret);
        com_assert_rv(ctx->pic_ccsao[0] != NULL, ret);
    }
    if (ctx->pic_ccsao[1] == NULL)
    {
        ctx->pic_ccsao[1] = com_pic_alloc(&ctx->rpm.pa, &ret);
        com_assert_rv(ctx->pic_ccsao[1] != NULL, ret);
    }
#else
    if (ctx->pic_ccsao == NULL)
    {
        ctx->pic_ccsao = com_pic_alloc(&ctx->rpm.pa, &ret);
        com_assert_rv(ctx->pic_ccsao != NULL, ret);
    }
#endif
#endif
    if (ctx->pic_alf_Org == NULL)
    {
        ctx->pic_alf_Org = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_alf_Org != NULL, ret);
    }
    if (ctx->pic_alf_Rec == NULL)
    {
        ctx->pic_alf_Rec = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_alf_Rec != NULL, ret);
    }

    decide_slice_type(ctx);
    ctx->lcu_cnt = ctx->info.f_lcu;
    ctx->slice_num = 0;
    if(ctx->slice_type == SLICE_I) ctx->last_intra_ptr = ctx->ptr;
    size = sizeof(s8) * ctx->info.f_scu * REFP_NUM;
    com_mset_x64a(ctx->map.map_refi, -1, size);
    size = sizeof(s16) * ctx->info.f_scu * REFP_NUM * MV_D;
    com_mset_x64a(ctx->map.map_mv, 0, size);
    /* initialize bitstream container */
    com_bsw_init(&ctx->bs, bitb->addr, bitb->addr2, bitb->bsize, NULL);
    /* clear map */
    com_mset_x64a(ctx->map.map_scu, 0, sizeof(u32) * ctx->info.f_scu);
    com_mset_x64a(ctx->map.map_cu_mode, 0, sizeof(u32) * ctx->info.f_scu);
#if USE_SP
    com_mset_x64a(ctx->map.map_usp, 0, sizeof(u8) * ctx->info.f_scu);
#endif

#if LIBVC_ON
    // this part determine whether the Ipic references library picture.
    int is_RLpic_flag = 0;
    if (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing && ctx->slice_type == SLICE_I)
    {
        for (int i = 0; i < ctx->rpm.libvc_data->num_RLpic; i++)
        {
            if (ctx->poc == ctx->rpm.libvc_data->list_poc_of_RLpic[i])
            {
                is_RLpic_flag = 1;
                break;
            }
        }
    }
    ctx->is_RLpic_flag = is_RLpic_flag;
#endif

#if ALO
    if (ctx->param.alo_enable_type == 1)
    {
        if (ctx->param.i_period <= 0 && ctx->poc != 0)
        {
            if (ctx->poc % GOP_P == 0)
            {
                updateOrgPic(ctx);
                updateCosineSimilarity(ctx);
                updateCtuFactor(ctx);
            }
        }
        else if (ctx->param.i_period > 1 && ctx->poc % ctx->param.i_period != 0)
        {
            if (ctx->poc % ctx->param.gop_size == 0)
            {
                updateOrgPic(ctx);
                updateCosineSimilarity(ctx);
                updateCtuFactor(ctx);
            }
        }
    }
#endif

    return COM_OK;
}

int enc_pic_finish(ENC_CTX *ctx, COM_BITB *bitb, ENC_STAT *stat)
{
    COM_IMGB *imgb_o, *imgb_c;
    COM_BSW  *bs;
    int        ret;
    int        i, j;
    int ph_size;
    bs   = &ctx->bs;

    demulate(bs);
    /*encode patch end*/
    ret = enc_eco_send(bs);
    com_assert_rv(ret == COM_OK, ret);
    /* de-init BSW */
    com_bsw_deinit(bs);
    /* ending */

    /* expand current encoding picture, if needs */
    enc_picbuf_expand(ctx, PIC_REC(ctx));

    /* picture buffer management */
#if LIBVC_ON
    if (ctx->info.sqh.library_stream_flag)
    {
        ret = com_picman_put_libpic(&ctx->rpm, PIC_REC(ctx), ctx->slice_type,
                                    ctx->ptr, ctx->info.pic_header.decode_order_index, ctx->temporal_id, 1, ctx->refp, &ctx->info.pic_header);
    }
    else
#endif
    {
        ret = com_picman_put_pic(&ctx->rpm, PIC_REC(ctx), ctx->slice_type,
                                 ctx->ptr, ctx->info.pic_header.decode_order_index, 
#if OBMC
#if CUDQP
                                 ctx->info.pic_header.picture_output_delay, ctx->temporal_id, 0, ctx->refp, &(ctx->info.pic_header));
#else
                                 ctx->info.pic_header.picture_output_delay, ctx->temporal_id, 0, ctx->refp, ctx->info.pic_header.picture_qp);
#endif
#else
                                 ctx->info.pic_header.picture_output_delay, ctx->temporal_id, 0, ctx->refp);
#endif
    }
    com_assert_rv(ret == COM_OK, ret);
    imgb_o = PIC_ORG(ctx)->imgb;
    com_assert(imgb_o != NULL);
    imgb_c = PIC_REC(ctx)->imgb;
    com_assert(imgb_c != NULL);
    /* set stat */
    ph_size = stat->write;
    com_mset(stat, 0, sizeof(ENC_STAT));
    stat->write = ph_size + COM_BSW_GET_WRITE_BYTE(bs);
    stat->ctype = COM_CT_PICTURE;
    stat->stype = ctx->slice_type;
    stat->fnum  = ctx->pic_cnt;
    stat->qp    = ctx->info.pic_header.picture_qp;
    stat->poc   = ctx->param.gop_size == 1? ctx->pic_cnt : ctx->ptr; //for AI and IPPP cases, gop_size = 1; for RA, LDB, LDP, gop_size > 1
#if LIBVC_ON
    stat->is_RLpic_flag = ctx->is_RLpic_flag;
#endif
    for(i = 0; i < 2; i++)
    {
        stat->refpic_num[i] = ctx->rpm.num_refp[i];
        for(j = 0; j < stat->refpic_num[i]; j++)
        {
#if LIBVC_ON
            if (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing && ctx->refp[j][i].is_library_picture)
            {
#if IPPPCRR
                stat->reflib[i][j] = 1;
#endif
                stat->refpic[i][j] = ctx->refp[j][i].pic->ptr;
            }
            else
#endif
            {
#if IPPPCRR
                stat->reflib[i][j] = 0;
#endif
                stat->refpic[i][j] = ctx->refp[j][i].ptr;
            }
        }
    }
    ctx->pic_cnt++; /* increase picture count */
    ctx->param.f_ifrm = 0; /* clear force-IDR flag */
    ctx->pico->is_used = 0;
    imgb_c->ts[0] = bitb->ts[0] = imgb_o->ts[0];
    imgb_c->ts[1] = bitb->ts[1] = imgb_o->ts[1];
    imgb_c->ts[2] = bitb->ts[2] = imgb_o->ts[2];
    imgb_c->ts[3] = bitb->ts[3] = imgb_o->ts[3];
    if(imgb_o) imgb_o->release(imgb_o);
    return COM_OK;
}

int enc_pic(ENC_CTX * ctx, COM_BITB * bitb, ENC_STAT * stat)
{
    ENC_CORE * core;
    COM_BSW   * bs;
    COM_PIC_HEADER    * pic_header;
    COM_SQH * sqh;
    COM_SH_EXT * shext;
    COM_CNKH    cnkh;
    int          ret;
    int          i;
    s8 * map_delta_qp;
    int last_lcu_qp;
    int last_lcu_delta_qp;
    bs = &ctx->bs;
    core = ctx->core;
    pic_header = &ctx->info.pic_header;
    sqh = &ctx->info.sqh;
    shext = &ctx->info.shext;
    u8 patch_idx=0;
#if PATCH
    PATCH_INFO *patch;
    patch = ctx->patch;
    int patch_cur_lcu_x;
    int patch_cur_lcu_y;
#if USE_SP
    u8 * map_usp_temp;
#endif
    u32 * map_scu_temp;
    s8  (*map_refi_temp)[REFP_NUM];
    s16 (*map_mv_temp)[REFP_NUM][MV_D];
    u32 *map_cu_mode_temp;
    map_scu_temp = ctx->map.map_scu_temp;
    map_refi_temp = ctx->map.map_refi_temp;
    map_mv_temp = ctx->map.map_mv_temp;
    map_cu_mode_temp = ctx->map.map_cu_mode_temp;
#if USE_SP
    map_usp_temp = ctx->map.map_usp_temp;
#endif
    com_mset_x64a(map_scu_temp, 0, sizeof(u32)* ctx->info.f_scu);
    com_mset_x64a(map_refi_temp, -1, sizeof(s8)* ctx->info.f_scu * REFP_NUM);
    com_mset_x64a(map_mv_temp, 0, sizeof(s16) * ctx->info.f_scu * REFP_NUM * MV_D);
    com_mset_x64a(map_cu_mode_temp, 0, sizeof(u32) * ctx->info.f_scu);
#if USE_SP
    com_mset_x64a(map_usp_temp, 0, sizeof(u8)* ctx->info.f_scu);
#endif
    int patch_cur_index = -1;
    int num_of_patch = patch->columns *patch->rows;
    s8  *patch_sao_enable = com_malloc(sizeof(s8)*num_of_patch*N_C);

    for( int i = 0; i < num_of_patch; i++ )
    {
        for( int j = 0; j < N_C; j++ )
            *(patch_sao_enable + i*N_C + j) = 1;
    }
#if PATCH_HEADER_PARAM_TEST
    //for test each patch has different sao param
    for( int i = 0; i < num_of_patch; i++ )
    {
        *(patch_sao_enable + i*N_C + Y_C) = (i / 4) % 2;
        *(patch_sao_enable + i*N_C + U_C) = (i / 2) % 2;
        *(patch_sao_enable + i*N_C + V_C) = i % 2;
    }
#endif
#endif
#if HLS_RPL
    /* Set slice header */
    //This needs to be done before reference picture marking and reference picture list construction are invoked
    set_pic_header(ctx, pic_header);
    set_sh(ctx, shext, patch_sao_enable + (patch_cur_index+1)*N_C);

    ctx->wq[0] = pic_header->wq_4x4_matrix;
    ctx->wq[1] = pic_header->wq_8x8_matrix;

    if (pic_header->slice_type != SLICE_I && pic_header->poc != 0) //TODO: perhaps change this condition to say that if this slice is not a slice in IDR picture
    {
        ret = create_explicit_rpl(&ctx->rpm, pic_header);
        if (ret == 1)
        {
            if (pic_header->rpl_l0_idx == -1)
                pic_header->ref_pic_list_sps_flag[0] = 0;
            if (pic_header->rpl_l1_idx == -1)
                pic_header->ref_pic_list_sps_flag[1] = 0;
        }
    }

#if LIBVC_ON
#if !CRR_ENC_OPT_CFG
#if IPPPCRR
#if PB_REF_LIB
#if LIB_PIC_UPDATE
    if (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing && (ctx->slice_type == SLICE_P || ctx->slice_type == SLICE_B) && ctx->rpm.libvc_data->lib_pic_update)
#else
    if (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing && (ctx->slice_type == SLICE_P || ctx->slice_type == SLICE_B) && ctx->rpm.libvc_data->first_pic_as_libpic)
#endif
    {
        pic_header->rpl_l0_idx  = -1;
        pic_header->ref_pic_list_sps_flag[0]  = 0;
        pic_header->num_ref_idx_active_override_flag = 1;
        pic_header->rpl_l0.ref_pic_active_num ++;
        pic_header->rpl_l0.reference_to_library_enable_flag = 1;
        int idx = pic_header->rpl_l0.ref_pic_num;
        pic_header->rpl_l0.ref_pics[idx] = ctx->rpm.libvc_data->num_lib_pic - 1;
        pic_header->rpl_l0.ref_pics_ddoi[idx] = ctx->rpm.libvc_data->num_lib_pic - 1;
        pic_header->rpl_l0.library_index_flag[idx] = 1;
        pic_header->rpl_l0.ref_pic_num++;
    }
#endif
#endif
#endif

#if CRR_ENC_OPT_CFG
    if (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing)
    {
        if (ctx->is_RLpic_flag)
        {
            ctx->rpm.libvc_data->libpic_idx = get_libidx(ctx->rpm.libvc_data, ctx->ptr);
            if (ctx->rpm.libvc_data->rl_ref_lib > 1)
            {
                ctx->slice_type = SLICE_B;
            }
            else
                if (ctx->rpm.libvc_data->rl_ref_lib == 1)
                {
                    ctx->slice_type = SLICE_P;
                }
            pic_header->slice_type = ctx->slice_type;
        }
        if (((ctx->rpm.libvc_data->libpic_idx >= 0) && (ctx->rpm.libvc_data->pb_ref_lib && ctx->slice_type != SLICE_I && !ctx->is_RLpic_flag)) || (ctx->rpm.libvc_data->rl_ref_lib && ctx->is_RLpic_flag))
        {
            pic_header->rpl_l0_idx = pic_header->rpl_l1_idx = -1;
            pic_header->ref_pic_list_sps_flag[0] = pic_header->ref_pic_list_sps_flag[1] = 0;
            if (((pic_header->rpl_l0.ref_pic_active_num < ctx->rpm.libvc_data->max_list_refnum) && ctx->rpm.libvc_data->lib_in_l0 > 0) || ctx->is_RLpic_flag)
            {
                pic_header->num_ref_idx_active_override_flag = 1;
                pic_header->rpl_l0.ref_pic_active_num ++;
                pic_header->rpl_l0.reference_to_library_enable_flag = 1;
                if (ctx->rpm.libvc_data->lib_in_l0 == 1 || ctx->is_RLpic_flag)
                {
                    for (int ii = pic_header->rpl_l0.ref_pic_num; ii > 0; ii--)
                    {
                        pic_header->rpl_l0.ref_pics[ii] = pic_header->rpl_l0.ref_pics[ii - 1];
                        pic_header->rpl_l0.ref_pics_ddoi[ii] = pic_header->rpl_l0.ref_pics_ddoi[ii - 1];
                        pic_header->rpl_l0.library_index_flag[ii] = pic_header->rpl_l0.library_index_flag[ii - 1];
                    }
                    pic_header->rpl_l0.ref_pics[0] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l0.ref_pics_ddoi[0] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l0.library_index_flag[0] = 1;
                    pic_header->rpl_l0.ref_pic_num++;
                }
                else
                {
                    int idx = pic_header->rpl_l0.ref_pic_num;
                    pic_header->rpl_l0.ref_pics[idx] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l0.ref_pics_ddoi[idx] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l0.library_index_flag[idx] = 1;
                    pic_header->rpl_l0.ref_pic_num++;
                }
            }
            if (((pic_header->rpl_l1.ref_pic_active_num < ctx->rpm.libvc_data->max_list_refnum) && ctx->rpm.libvc_data->lib_in_l1 > 0) || (ctx->rpm.libvc_data->rl_ref_lib>1 && ctx->is_RLpic_flag))
            {
                pic_header->num_ref_idx_active_override_flag = 1;
                pic_header->rpl_l1.ref_pic_active_num ++;
                pic_header->rpl_l1.reference_to_library_enable_flag = 1;
                if (ctx->rpm.libvc_data->lib_in_l1 == 1 || ctx->is_RLpic_flag)
                {
                    for (int ii = pic_header->rpl_l1.ref_pic_num; ii > 0; ii--)
                    {
                        pic_header->rpl_l1.ref_pics[ii] = pic_header->rpl_l1.ref_pics[ii - 1];
                        pic_header->rpl_l1.ref_pics_ddoi[ii] = pic_header->rpl_l1.ref_pics_ddoi[ii - 1];
                        pic_header->rpl_l1.library_index_flag[ii] = pic_header->rpl_l1.library_index_flag[ii - 1];
                    }
                    pic_header->rpl_l1.ref_pics[0] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l1.ref_pics_ddoi[0] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l1.library_index_flag[0] = 1;
                    pic_header->rpl_l1.ref_pic_num++;
                }
                else
                {
                    int idx = pic_header->rpl_l1.ref_pic_num;
                    pic_header->rpl_l1.ref_pics[idx] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l1.ref_pics_ddoi[idx] = ctx->rpm.libvc_data->libpic_idx;
                    pic_header->rpl_l1.library_index_flag[idx] = 1;
                    pic_header->rpl_l1.ref_pic_num++;
                }
            }
        }
    }
#else
        if (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing && ctx->is_RLpic_flag)
        {
#if IPPPCRR
#if LIB_PIC_UPDATE
        if (ctx->rpm.libvc_data->lib_pic_update)
#else
        if (ctx->rpm.libvc_data->first_pic_as_libpic)
#endif
            ctx->slice_type = SLICE_P;
        else
            ctx->slice_type = SLICE_B;
#else
        ctx->slice_type = SLICE_B;
#endif
        pic_header->slice_type = ctx->slice_type;

        //set rpl
        int libpic_idx;
        libpic_idx = get_libidx(ctx->rpm.libvc_data, ctx->ptr);
        com_assert_rv(libpic_idx >= 0, libpic_idx);

        pic_header->rpl_l0_idx = pic_header->rpl_l1_idx = -1;
        pic_header->ref_pic_list_sps_flag[0] = pic_header->ref_pic_list_sps_flag[1] = 0;

        pic_header->num_ref_idx_active_override_flag = 1;
        pic_header->rpl_l0.ref_pic_active_num = 1;
        pic_header->rpl_l0.reference_to_library_enable_flag = 1;
#if IPPPCRR
#if LIB_PIC_UPDATE
        if (!ctx->rpm.libvc_data->lib_pic_update)
#else
        if (!ctx->rpm.libvc_data->first_pic_as_libpic)
#endif
        {
            pic_header->rpl_l1.ref_pic_active_num = 1;
            pic_header->rpl_l1.reference_to_library_enable_flag = 1;
        }
#else
        pic_header->rpl_l1.ref_pic_active_num = 1;
        pic_header->rpl_l1.reference_to_library_enable_flag = 1;
#endif

        for (int ii = pic_header->rpl_l0.ref_pic_num; ii > 0; ii--)
        {
            pic_header->rpl_l0.ref_pics[ii] = pic_header->rpl_l0.ref_pics[ii - 1];
            pic_header->rpl_l0.ref_pics_ddoi[ii] = pic_header->rpl_l0.ref_pics_ddoi[ii - 1];
            pic_header->rpl_l0.library_index_flag[ii] = pic_header->rpl_l0.library_index_flag[ii - 1];
        }
        pic_header->rpl_l0.ref_pics[0] = libpic_idx;
        pic_header->rpl_l0.ref_pics_ddoi[0] = libpic_idx;
        pic_header->rpl_l0.library_index_flag[0] = 1;
        pic_header->rpl_l0.ref_pic_num++;
#if IPPPCRR
#if LIB_PIC_UPDATE
        if (!ctx->rpm.libvc_data->lib_pic_update)
#else
        if (!ctx->rpm.libvc_data->first_pic_as_libpic)
#endif
        {
#endif
            for (int jj = pic_header->rpl_l1.ref_pic_num; jj > 0; jj--)
            {
                pic_header->rpl_l1.ref_pics[jj] = pic_header->rpl_l1.ref_pics[jj - 1];
                pic_header->rpl_l1.ref_pics_ddoi[jj] = pic_header->rpl_l1.ref_pics_ddoi[jj - 1];
                pic_header->rpl_l1.library_index_flag[jj] = pic_header->rpl_l1.library_index_flag[jj - 1];
            }
            pic_header->rpl_l1.ref_pics[0] = libpic_idx;
            pic_header->rpl_l1.ref_pics_ddoi[0] = libpic_idx;
            pic_header->rpl_l1.library_index_flag[0] = 1;
            pic_header->rpl_l1.ref_pic_num++;
#if IPPPCRR
        }
#endif
    }
#endif
#endif

#if CABAC_MULTI_PROB
    if (pic_header->slice_type == SLICE_I)
    {
        mCabac_ws = MCABAC_SHIFT_I;
        mCabac_offset = (1 << (mCabac_ws - 1));
        counter_thr1 = 0;
        counter_thr2 = COUNTER_THR_I;
    }
    else if (pic_header->slice_type == SLICE_B)
    {
        mCabac_ws = MCABAC_SHIFT_B;
        mCabac_offset = (1 << (mCabac_ws - 1));
        counter_thr1 = 3;
        counter_thr2 = COUNTER_THR_B;
    }
    else
    {
        mCabac_ws = MCABAC_SHIFT_P;
        mCabac_offset = (1 << (mCabac_ws - 1));
        counter_thr1 = 3;
        counter_thr2 = COUNTER_THR_P;
    }

    if (ctx->info.sqh.mcabac_enable_flag)
    {
        g_compatible_back = 0;
    }
    else
    {
        g_compatible_back = 1;
    }
#endif

    if ((pic_header->decode_order_index%DOI_CYCLE_LENGTH) < g_DOIPrev)
    {
        com_picman_dpbpic_doi_minus_cycle_length(&ctx->rpm);
    }
    g_DOIPrev = pic_header->decode_order_index%DOI_CYCLE_LENGTH;
#if LIBVC_ON
#if IPPPCRR&&LIB_PIC_UPDATE
    if (ctx->rpm.libvc_data->lib_pic_update)
    {
        if (!ctx->rpm.libvc_data->is_libpic_processing)
        {
            /* reference picture marking */
            ret = com_picman_refpic_marking(&ctx->rpm, pic_header);
            com_assert_rv(ret == COM_OK, ret);
        }
    }
    else
#endif
        if (!sqh->library_stream_flag)    
        {
        /* reference picture marking */
        ret = com_picman_refpic_marking(&ctx->rpm, pic_header);
        com_assert_rv(ret == COM_OK, ret);
    }
#endif

    com_constrcut_ref_list_doi(pic_header);

    /* reference picture lists construction */
    ret = com_picman_refp_rpl_based_init(&ctx->rpm, pic_header, ctx->refp);
#else
    /* initialize reference pictures */
    ret = com_picman_refp_init(&ctx->rpm, ctx->info.sqh.num_ref_pics_act, ctx->slice_type, ctx->ptr, ctx->temporal_id, ctx->last_intra_ptr, ctx->refp);
#endif
    com_assert_rv(ret == COM_OK, ret);
#if INTER_ME_MVLIB
    BOOL isLD = TRUE;
    for (int list = 0; list < 2 && isLD; list++)
    {
        int numRef = ctx->rpm.num_refp[list];
        for (int refIdx = 0; refIdx < numRef; refIdx++)
        {
            if (ctx->refp[refIdx][list].ptr > pic_header->poc)
            {
                isLD = FALSE;
                break;
            }
        }
    }
#endif
    /* initialize mode decision for frame encoding */
    ret = enc_mode_init_frame(ctx);
    com_assert_rv(ret == COM_OK, ret);

    /* slice layer encoding loop */
    core->x_lcu = core->y_lcu = 0;
    core->x_pel = core->y_pel = 0;
    core->lcu_num = 0;
    ctx->lcu_cnt = ctx->info.f_lcu;
    /* Set chunk header */
    set_cnkh(ctx, &cnkh, COM_VER_1, COM_CT_PICTURE);

    /* initialize entropy coder */
    enc_sbac_init(bs);
    com_sbac_ctx_init(&(GET_SBAC_ENC(bs)->ctx));
    com_sbac_ctx_init(&(core->s_curr_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2].ctx));
    core->bs_temp.pdata[1] = &core->s_temp_run;
    /* LCU encoding */
#if TRACE_RDO_EXCLUDE_I
    if(ctx->slice_type != SLICE_I)
    {
#endif
        COM_TRACE_SET(0);
#if TRACE_RDO_EXCLUDE_I
    }
#endif

    last_lcu_qp = ctx->info.shext.slice_qp;
    last_lcu_delta_qp = 0;
    map_delta_qp = ctx->map.map_delta_qp;
#if PATCH
    /*initial patch info*/
    patch->x_pel = patch->x_pat = patch->y_pel = patch->y_pat = patch->idx = 0;
    patch_cur_lcu_x = 0;
    patch_cur_lcu_y = 0;
    patch->left_pel = patch->x_pel;
    patch->up_pel = patch->y_pel;
    patch->right_pel = patch->x_pel + (*(patch->width_in_lcu + patch->x_pat) << ctx->info.log2_max_cuwh);
    patch->down_pel = patch->y_pel + (*(patch->height_in_lcu + patch->y_pat) << ctx->info.log2_max_cuwh);
#endif
#if UMVE_ENH
    ctx->dataCol = FALSE;
#endif
#if INTER_ME_MVLIB
    resetUniMvList(core, isLD);
#endif
#if ENC_ME_IMP
    resetAffUniMvList(core, isLD);
#endif
#if EPMC
    chroma_res_analyze(ctx,&(ctx->fcbcr));
    int modeIdx;
    if (MOD_IDX == 1)
    {
        modeIdx = 0;
    }
    else if (MOD2_IDX == 1)
    {
        modeIdx = 1;
    }
    else
    {
        assert(0);
    }
    if (ctx->fcbcr > TH_CBCR) //-0.5 model
    {
        ctx->info.pic_header.ph_epmc_model_flag = modeIdx;
    }
    else
    {
        ctx->info.pic_header.ph_epmc_model_flag = 1-modeIdx;
    }
#endif
#if IPC
    if (ctx->slice_type != SLICE_I)
    {
        ctx->info.pic_header.ph_ipc_flag = enc_ipc_pic(ctx, core, ctx->refp);
    }
#endif
    while(1)
    {

        int lcu_qp;
        int adj_qp_cb, adj_qp_cr;

#if PATCH
        /*set patch idx*/
        if (patch_cur_index != patch->idx)
            enc_set_patch_idx(ctx->map.map_patch_idx, patch, ctx->info.pic_width, ctx->info.pic_height);
        patch_cur_index = patch->idx;
#endif

#if IPPPCRR && LIB_PIC_UPDATE
        if (ctx->rpm.libvc_data->encode_skip==0)
        {
#endif
            // reset HMVP list before each LCU line for parallel computation
#if PATCH
            if (core->x_lcu * ctx->info.max_cuwh == patch->left_pel)
            {
#else
            if (core->x_lcu == 0)
            {
#endif
                core->cnt_hmvp_cands = 0;
                com_mset_x64a(core->motion_cands, 0, sizeof(COM_MOTION)*ALLOWED_HMVP_NUM);
#if IBC_BVP
                // reset HBVP list before each LCU line for parallel computation
                core->cnt_hbvp_cands = 0;
                com_mset_x64a(core->block_motion_cands, 0, sizeof(COM_BLOCK_MOTION)*ALLOWED_HBVP_NUM);
#endif
#if USE_SP
                core->n_offset_num = 0;
                com_mset_x64a(core->n_recent_offset, 0, sizeof(COM_MOTION)*SP_RECENT_CANDS);
#endif
                core->n_pv_num = 0;
                com_mset_x64a(core->n_recent_pv, 0, sizeof(COM_MOTION)*MAX_SRB_PRED_SIZE);
            }

            if (ctx->info.shext.fixed_slice_qp_flag)
            {
                lcu_qp = ctx->info.shext.slice_qp;
            }
#if CUDQP
            else if (com_is_cu_dqp(&ctx->info))
            {
                *map_delta_qp++ = 0;
                lcu_qp = last_lcu_qp;
                assert(last_lcu_qp == ctx->info.shext.slice_qp);
            }
#endif
            else
            {
                //for testing delta QP
                int test_delta_qp = (((core->x_lcu + 1) * 5 + core->y_lcu * 3 + ctx->ptr * 6) % 16) + ((((core->x_lcu + 2) * 3 + core->y_lcu * 5 + ctx->ptr) % 8) << 4); //lower 4 bits and higher 3 bits
                int max_abs_delta_qp = 32 + (ctx->info.bit_depth_internal - 8) * 4;
                lcu_qp = last_lcu_qp + (test_delta_qp % (max_abs_delta_qp * 2 + 1)) - max_abs_delta_qp;
                lcu_qp = COM_CLIP3(MIN_QUANT, (MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth), lcu_qp);
                *map_delta_qp++ = (s8)(lcu_qp - last_lcu_qp);
                last_lcu_qp = lcu_qp;
            }
#if CUDQP
            ctx->cu_qp_group.pred_qp = lcu_qp; //when cudqp is on, it will be updated at gp group level
#endif

            core->qp_y = lcu_qp;
            adj_qp_cb = core->qp_y + pic_header->chroma_quant_param_delta_cb - ctx->info.qp_offset_bit_depth;
            adj_qp_cr = core->qp_y + pic_header->chroma_quant_param_delta_cr - ctx->info.qp_offset_bit_depth;
#if PMC || EPMC
            int adj_qp_cr_pmc = adj_qp_cr + V_QP_OFFSET;
            adj_qp_cr_pmc = COM_CLIP(adj_qp_cr_pmc, MIN_QUANT - 16, MAX_QUANT_BASE);
            if (adj_qp_cr_pmc >= 0)
            {
                adj_qp_cr_pmc = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, adj_qp_cr_pmc)];
            }
            core->qp_v_pmc = COM_CLIP(adj_qp_cr_pmc + ctx->info.qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth);
#endif
            adj_qp_cb = COM_CLIP(adj_qp_cb, MIN_QUANT - 16, MAX_QUANT_BASE);
            adj_qp_cr = COM_CLIP(adj_qp_cr, MIN_QUANT - 16, MAX_QUANT_BASE);
            if (adj_qp_cb >= 0)
            {
                adj_qp_cb = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, adj_qp_cb)];
            }
            if (adj_qp_cr >= 0)
            {
                adj_qp_cr = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, adj_qp_cr)];
            }
            core->qp_u = COM_CLIP(adj_qp_cb + ctx->info.qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth);
            core->qp_v = COM_CLIP(adj_qp_cr + ctx->info.qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth);

            lcu_qp -= ctx->info.qp_offset_bit_depth; // calculate lambda for 8-bit distortion
            ctx->lambda[0] = 1.43631 * pow(2.0, (lcu_qp - 16.0) / 4.0);
#if ALO
            if (ctx->param.alo_enable_type == 1)
            {
                if (ctx->param.i_period <= 0)
                {
                    if (ctx->slice_type != SLICE_I && ctx->poc % GOP_P == 0)
                    {
                        ctx->lambda[0] = adjustCtuLambda(ctx);
                    }
                }
                else if (ctx->param.i_period > 1)
                {
                    if (ctx->slice_type != SLICE_I && ctx->poc % ctx->param.gop_size == 0)
                    {
                        ctx->lambda[0] = adjustCtuLambda(ctx);
                    }
                }
            }
#endif
            ctx->dist_chroma_weight[0] = pow(2.0, (lcu_qp - adj_qp_cb) / 4.0);
            ctx->dist_chroma_weight[1] = pow(2.0, (lcu_qp - adj_qp_cr) / 4.0);

            ctx->lambda[1] = ctx->lambda[0] / ctx->dist_chroma_weight[0];
            ctx->lambda[2] = ctx->lambda[0] / ctx->dist_chroma_weight[1];
            ctx->sqrt_lambda[0] = sqrt(ctx->lambda[0]);
            ctx->sqrt_lambda[1] = sqrt(ctx->lambda[1]);
            ctx->sqrt_lambda[2] = sqrt(ctx->lambda[2]);

#if PMC || EPMC
            ctx->dist_chroma_weight_v_pmc = pow(2.0, (lcu_qp - adj_qp_cr_pmc) / 4.0);
            ctx->lambda_v_pmc = ctx->lambda[0] / ctx->dist_chroma_weight_v_pmc;
#endif

            /* initialize structures *****************************************/
            ret = enc_mode_init_lcu(ctx, core);
            com_assert_rv(ret == COM_OK, ret);
#if USE_SP
            if (ctx->info.pic_header.sp_pic_flag)
            {
                sp_lcu_hash_save(ctx->sp_encoder, core->x_pel, core->y_pel);
            }
#endif
            /* mode decision *************************************************/
            SBAC_LOAD(core->s_curr_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2], *GET_SBAC_ENC(bs));
            core->s_curr_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2].is_bitcount = 1;
            enc_init_bef_data(core, ctx);

            SBAC_STORE(core->s_sao_init, core->s_curr_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2]);

#if FS_ALL_COMBINATION
            derive_split_combination_CTU(ctx, core);
#endif
#if VERIFY_SPLIT
            printf("\nCtu_idx %5d\tSplit_comb %d %d %d %d %d %d    ", ctx->slice_type == SLICE_I ? ctx->ctu_idx_in_seq_I_slice_ctu : ctx->ctu_idx_in_seq_B_slice_ctu,
                ctx->split_combination[0], ctx->split_combination[1], ctx->split_combination[2], ctx->split_combination[3], ctx->split_combination[4], ctx->split_combination[5]);
#endif
#if USE_IBC
            if (ctx->info.pic_header.ibc_flag && (ctx->param.ibc_fast_method & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
            {
                reset_ibc_search_range(ctx, core->x_pel, core->y_pel, ctx->info.max_cuwh, ctx->info.max_cuwh);
            }
#endif
#if IPPPCRR&&LIB_PIC_UPDATE
        }
#endif
#if USE_SP
        if (ctx->param.evs_ubvs_enable_flag)
        {
            ctx->map_cu_data[core->lcu_num].p_SRB = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * N_C * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].pvbuf_reused_flag = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].p_SRB_prev = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * N_C * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].all_comp_flag = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].all_comp_pre_flag = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].cuS_flag = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].cuS_pre_flag = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].pv_x = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].pv_x_prev = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].pv_y = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].pv_y_prev = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].srb_u = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].srb_v = (s16*)malloc(sizeof(s16) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].m_dpb_reYonly = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].m_dpb_reYonly_prev = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
            ctx->map_cu_data[core->lcu_num].m_dpb_idx = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_SIZE);
            ctx->map_cu_data[core->lcu_num].m_dpb_idx_prev = (u8*)malloc(sizeof(u8) * MAX_CU_CNT_IN_LCU * MAX_SRB_PRED_SIZE);
        }
#endif
        ret = enc_mode_analyze_lcu(ctx, core);
#if IPPPCRR&&LIB_PIC_UPDATE
        if (ctx->rpm.libvc_data->encode_skip==0)
        {
#endif
#if FIXED_SPLIT
            if ((core->x_lcu + 1) * ctx->info.max_cuwh <= ctx->info.pic_width && (core->y_lcu + 1) * ctx->info.max_cuwh <= ctx->info.pic_height)
            {
                ctx->ctu_idx_in_picture++;
                ctx->ctu_idx_in_sequence++;
                ctx->ctu_idx_in_seq_I_slice_ctu += ctx->slice_type == SLICE_I;
                ctx->ctu_idx_in_seq_B_slice_ctu += ctx->slice_type != SLICE_I;
            }
#endif
            com_assert_rv(ret == COM_OK, ret);
            /* entropy coding ************************************************/
            //write to bitstream
            if (ctx->info.sqh.sample_adaptive_offset_enable_flag)
            {
                int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
                write_param_sao_one_lcu(ctx, core->y_pel, core->x_pel, ctx->sao_blk_params[lcu_pos]);
            }
#if USE_SP
            if (ctx->info.pic_header.evs_ubvs_pic_flag)
            {
                assert(core->n_pv_num_last <= MAX_SRB_PRED_SIZE);
                for (int i = 0; i < 3; i++)
                {
                    copy_fap_unpred_pix_motion_table(core->n_recent_pv[i], &core->n_pv_num, core->n_recent_pv_last[i], core->n_pv_num_last);
                }
                memcpy(core->n_recent_all_comp_flag, core->n_recent_allcomflag_last, core->n_pv_num * sizeof(u8));
                memcpy(core->n_recent_cuS_flag, core->n_recent_cuSflag_last, core->n_pv_num * sizeof(u8));
                memcpy(core->n_recent_pv_x, core->n_recent_pv_x_last, core->n_pv_num * sizeof(s16));
                memcpy(core->n_recent_pv_y, core->n_recent_pv_y_last, core->n_pv_num * sizeof(s16));
                memcpy(core->n_recent_dpb_reYonly, core->n_recent_dpb_reYonly_last, core->n_pv_num * sizeof(u8));
                memcpy(core->n_recent_dpb_idx, core->n_recent_dpb_idx_last, core->n_pv_num * sizeof(u8));
            }
#endif
            ret = enc_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->info.max_cuwh, ctx->info.max_cuwh, 0
                , NO_SPLIT, 0, 0, NO_MODE_CONS, TREE_LC);
            com_assert_rv(ret == COM_OK, ret);
#if USE_SP
            if (ctx->info.pic_header.evs_ubvs_pic_flag)
            {
                assert(core->n_pv_num <= MAX_SRB_PRED_SIZE);
                for (int i = 0; i < 3; i++)
                {
                    copy_fap_unpred_pix_motion_table(core->n_recent_pv_last[i], &core->n_pv_num_last, core->n_recent_pv[i], core->n_pv_num);
                }
                memcpy(core->n_recent_allcomflag_last, core->n_recent_all_comp_flag, core->n_pv_num_last * sizeof(u8));
                memcpy(core->n_recent_cuSflag_last, core->n_recent_cuS_flag, core->n_pv_num_last * sizeof(u8));
                memcpy(core->n_recent_pv_x_last, core->n_recent_pv_x, core->n_pv_num_last * sizeof(s16));
                memcpy(core->n_recent_pv_y_last, core->n_recent_pv_y, core->n_pv_num_last * sizeof(s16));
                memcpy(core->n_recent_dpb_reYonly_last, core->n_recent_dpb_reYonly, core->n_pv_num_last * sizeof(u8));
                memcpy(core->n_recent_dpb_idx_last, core->n_recent_dpb_idx, core->n_pv_num_last * sizeof(u8));
            }
#endif
            /* prepare next step *********************************************/
#if IPPPCRR&&LIB_PIC_UPDATE
        }
#endif
        core->x_lcu++;
#if PATCH
        patch_cur_index = patch->idx;
        if (core->x_lcu >= *(patch->width_in_lcu + patch->x_pat) + patch_cur_lcu_x)
        {
            core->x_lcu = patch_cur_lcu_x;
            core->y_lcu++;
            if (core->y_lcu >= *(patch->height_in_lcu + patch->y_pat) + patch_cur_lcu_y)
            {
                // a new patch starts
                core->cnt_hmvp_cands = 0;
                com_mset_x64a(core->motion_cands, 0, sizeof(COM_MOTION)*ALLOWED_HMVP_NUM);
#if BGC
                com_mset_x64a(core->bgc_flag_cands, 0, sizeof(s8)*ALLOWED_HMVP_NUM);
                com_mset_x64a(core->bgc_idx_cands, 0, sizeof(s8)*ALLOWED_HMVP_NUM);
#endif
#if IBC_BVP
                core->cnt_hbvp_cands = 0;
                com_mset_x64a(core->block_motion_cands, 0, sizeof(COM_BLOCK_MOTION)*ALLOWED_HBVP_NUM);
#endif
#if USE_SP
                core->p_offset_num = 0;
                com_mset_x64a(core->parent_offset, 0, sizeof(COM_MOTION) * SP_MAX_SPS_CANDS); 
                core->b_offset_num = 0;
                com_mset_x64a(core->brother_offset, 0, sizeof(COM_MOTION) * SP_MAX_SPS_CANDS); 
                core->n_offset_num = 0;
                com_mset_x64a(core->n_recent_offset, 0, sizeof(COM_MOTION) * SP_RECENT_CANDS); 
#endif
                core->n_pv_num = 0;
                com_mset_x64a(core->n_recent_pv, 0, sizeof(COM_MOTION)*MAX_SRB_PRED_SIZE);
                enc_eco_slice_end_flag(bs, 1);
                enc_sbac_finish(bs, 0);
                /*update and store map_scu*/
                en_copy_lcu_scu(map_scu_temp, ctx->map.map_scu, map_refi_temp, ctx->map.map_refi, map_mv_temp, ctx->map.map_mv, map_cu_mode_temp, ctx->map.map_cu_mode, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP
                    ,map_usp_temp, ctx->map.map_usp
#endif
                );
                com_mset_x64a(ctx->map.map_scu, 0, sizeof(u32)* ctx->info.f_scu);
                com_mset_x64a(ctx->map.map_refi, -1, sizeof(s8)* ctx->info.f_scu * REFP_NUM);
                com_mset_x64a(ctx->map.map_mv, 0, sizeof(s16)* ctx->info.f_scu * REFP_NUM * MV_D);
                com_mset_x64a(ctx->map.map_cu_mode, 0, sizeof(u32) * ctx->info.f_scu);
#if USE_SP
                com_mset_x64a(ctx->map.map_usp, 0, sizeof(u8)* ctx->info.f_scu);
#endif
                /*set patch head*/
                if( patch_cur_index + 1 < num_of_patch )
                {
                    set_sh( ctx, shext, patch_sao_enable + (patch_cur_index + 1)*N_C );
                }
                /* initialize entropy coder */
                enc_sbac_init(bs);
                com_sbac_ctx_init(&(GET_SBAC_ENC(bs)->ctx));
                com_sbac_ctx_init(&(core->s_curr_best[ctx->info.log2_max_cuwh - 2][ctx->info.log2_max_cuwh - 2].ctx));

                last_lcu_qp = ctx->info.shext.slice_qp;
                last_lcu_delta_qp = 0;
                core->x_lcu = *(patch->width_in_lcu + patch->x_pat) + patch_cur_lcu_x;
                core->y_lcu = patch_cur_lcu_y;
                patch->x_pat++;
                if (core->x_lcu >= ctx->info.pic_width_in_lcu)
                {
                    core->x_lcu = 0;
                    core->y_lcu = *(patch->height_in_lcu + patch->y_pat) + patch_cur_lcu_y;
                    patch->y_pat++;
                    patch->x_pat = 0;
                }
                patch->idx++;
                patch_cur_lcu_x = core->x_lcu;
                patch_cur_lcu_y = core->y_lcu;
                core->x_pel = core->x_lcu << ctx->info.log2_max_cuwh;
                core->y_pel = core->y_lcu << ctx->info.log2_max_cuwh;
                patch->x_pel = core->x_pel;
                patch->y_pel = core->y_pel;
                /*reset the patch boundary*/
                patch->left_pel = patch->x_pel;
                patch->up_pel = patch->y_pel;
                patch->right_pel = patch->x_pel + (*(patch->width_in_lcu + patch->x_pat) << ctx->info.log2_max_cuwh);
                patch->down_pel = patch->y_pel + (*(patch->height_in_lcu + patch->y_pat) << ctx->info.log2_max_cuwh);
            }
        }
#else
        if (core->x_lcu >= ctx->info.pic_width_in_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }
#endif
        core->x_pel = core->x_lcu << ctx->info.log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->info.log2_max_cuwh;
#if PATCH
        core->lcu_num = core->x_lcu + core->y_lcu*ctx->info.pic_width_in_lcu;
#else
        core->lcu_num++;
#endif
        ctx->lcu_cnt--;
        /* end_of_picture_flag */
#if IPPPCRR && LIB_PIC_UPDATE
        if (ctx->rpm.libvc_data->encode_skip == 1)
        {
            if (ctx->lcu_cnt <= 0)
                break;
        }
        else
        {
#endif
            if (ctx->lcu_cnt > 0)
            {
#if PATCH
                if (patch_cur_index == patch->idx)
#endif
                    enc_eco_slice_end_flag(bs, 0);

            }
            else
            {
#if !PATCH
                enc_eco_slice_end_flag(bs, 1);
                enc_sbac_finish(bs, 0);
#endif
                break;
            }

#if IPPPCRR && LIB_PIC_UPDATE
        }
#endif
    }
#if IPPPCRR && LIB_PIC_UPDATE
    if (ctx->rpm.libvc_data->encode_skip == 0)
    {
#endif
#if PATCH
        /*get scu from storage*/
        patch->left_pel = 0;
        patch->up_pel = 0;
        patch->right_pel = ctx->info.pic_width;
        patch->down_pel = ctx->info.pic_height;
        en_copy_lcu_scu(ctx->map.map_scu, map_scu_temp, ctx->map.map_refi, map_refi_temp, ctx->map.map_mv, map_mv_temp, ctx->map.map_cu_mode, map_cu_mode_temp, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP
            ,ctx->map.map_usp, map_usp_temp
#endif
        );
        com_mset_x64a(map_scu_temp, 0, sizeof(u32)* ctx->info.f_scu);
        com_mset_x64a(map_refi_temp, -1, sizeof(s8)* ctx->info.f_scu * REFP_NUM);
        com_mset_x64a(map_mv_temp, 0, sizeof(s16)* ctx->info.f_scu * REFP_NUM * MV_D);
        com_mset_x64a(map_cu_mode_temp, 0, sizeof(u32)* ctx->info.f_scu);
#if USE_SP
        com_mset_x64a(map_usp_temp, 0, sizeof(u8)* ctx->info.f_scu);
#endif
        /*save sao enable param*/
        patch->patch_sao_enable = patch_sao_enable;
#endif
        /* deblocking filter */
        if (ctx->info.pic_header.loop_filter_disable_flag == 0)
        {
            ret = enc_deblock_avs2(ctx, PIC_REC(ctx)
#if DBR
                , PIC_ORG(ctx)
#endif
            );
            com_assert_rv(ret == COM_OK, ret);
        }
#if CCSAO
        if (ctx->info.sqh.ccsao_enable_flag)
        {
#if CCSAO_ENHANCEMENT
            copy_frame_for_ccsao(ctx->pic_ccsao[0], PIC_REC(ctx), Y_C);
            copy_frame_for_ccsao(ctx->pic_ccsao[0], PIC_REC(ctx), U_C);
            copy_frame_for_ccsao(ctx->pic_ccsao[0], PIC_REC(ctx), V_C);
#else
            copy_frame_for_ccsao(ctx->pic_ccsao, PIC_REC(ctx), Y_C);
#endif
        }
#endif

        /* sao filter */
        if (ctx->info.sqh.sample_adaptive_offset_enable_flag)
        {
            ret = enc_sao_avs2(ctx, PIC_REC(ctx));
            com_assert_rv(ret == COM_OK, ret);
        }

        /* esao filter */
#if ESAO
        if (ctx->info.sqh.esao_enable_flag)
        {
            ret = enc_esao(ctx, core);
            com_assert_rv(ret == COM_OK, ret);
        }
#endif

#if CCSAO
        /* ccsao filter */
        if (ctx->info.sqh.ccsao_enable_flag)
        {
            ret = enc_ccsao(ctx, core);
            com_assert_rv(ret == COM_OK, ret);
        }
#endif

        /* ALF*/
        if (ctx->info.sqh.adaptive_leveling_filter_enable_flag)
        {
            enc_sbac_init(&(core->bs_temp)); // init sbac for alf rdo
            ret = enc_alf_avs2(ctx, &(core->bs_temp), PIC_REC(ctx), PIC_ORG(ctx));
            com_assert_rv(ret == COM_OK, ret);
        }

        /* Bit-stream re-writing (START) */
        com_bsw_init(&ctx->bs, bitb->addr, bitb->addr2, bitb->bsize, NULL);

#if TRACE_RDO_EXCLUDE_I
        if (ctx->slice_type != SLICE_I)
        {
#endif
            COM_TRACE_SET(1);
#if TRACE_RDO_EXCLUDE_I
        }
#endif
#if REPEAT_SEQ_HEADER
        /* Encode Sequence header */
        int seq_header_size = 0;
#if LIBVC_ON
        //I-picture or RL-picture
        if (ctx->slice_type == SLICE_I || (ctx->info.sqh.library_picture_enable_flag && !ctx->rpm.libvc_data->is_libpic_processing && ctx->is_RLpic_flag))
#else
        if (ctx->slice_type == SLICE_I)
#endif
        {
            /* bitsteam initialize for sequence */
            ret = enc_eco_sqh(ctx, bs, sqh);
            com_assert_rv(ret == COM_OK, COM_ERR_INVALID_ARGUMENT);
#if EXTENSION_USER_DATA
            ret = enc_extension_and_user_data(ctx, bs, 0, 1, sqh, pic_header);
#endif

            /* de-init BSW */
            bs->fn_flush(bs);
            seq_header_size = COM_BSW_GET_WRITE_BYTE(bs);
            com_bsw_init(bs, bs->cur, bs->buftmp, ((int)(bs->end - bs->cur)) + 1, NULL);
        }
#endif

        /* Encode Picture header */
        ret = enc_eco_pic_header(bs, pic_header, sqh);
        com_assert_rv(ret == COM_OK, ret);
#if EXTENSION_USER_DATA
        enc_extension_and_user_data(ctx, bs, 1, 1, sqh, pic_header);  // encode entension_data after pic_header
#endif
#if EXTENSION_USER_DATA && WRITE_MD5_IN_USER_DATA
        if (ctx->param.use_pic_sign)
        {
            enc_extension_and_user_data(ctx, bs, 0, 0, sqh, pic_header);
        }
#endif
        bs->fn_flush(bs);
#if REPEAT_SEQ_HEADER
        stat->write = COM_BSW_GET_WRITE_BYTE(bs) + seq_header_size;
#else
        stat->write = COM_BSW_GET_WRITE_BYTE(bs);
#endif
        com_bsw_init(bs, bs->cur, bs->buftmp, ((int)(bs->end - bs->cur)) + 1, NULL);
#if PATCH
        /*initial patch info*/
        patch->x_pel = patch->x_pat = patch->y_pel = patch->y_pat = patch->idx = 0;
        patch_cur_lcu_x = 0;
        patch_cur_lcu_y = 0;
        /*SET the patch boundary*/
        patch->left_pel = patch->x_pel;
        patch->up_pel = patch->y_pel;
        patch->right_pel = patch->x_pel + (*(patch->width_in_lcu + patch->x_pat) << ctx->info.log2_max_cuwh);
        patch->down_pel = patch->y_pel + (*(patch->height_in_lcu + patch->y_pat) << ctx->info.log2_max_cuwh);
        patch_cur_index = -1;
#endif
        /* Encode patch header */
        ret = enc_eco_patch_header(bs, &ctx->info.sqh, pic_header, shext, patch_idx, patch);
        set_sh(ctx, shext, patch->patch_sao_enable + patch_idx * N_C);
        com_assert_rv(ret == COM_OK, ret);

        core->x_lcu = core->y_lcu = 0;
        core->x_pel = core->y_pel = 0;
        core->lcu_num = 0;
        ctx->lcu_cnt = ctx->info.f_lcu;
        for (i = 0; i < ctx->info.f_scu; i++)
        {
            MCU_CLR_CODED_FLAG(ctx->map.map_scu[i]);
        }
        enc_sbac_init(bs);
        com_sbac_ctx_init(&(GET_SBAC_ENC(bs)->ctx));
        /* Encode slice data */
        last_lcu_qp = ctx->info.shext.slice_qp;
        last_lcu_delta_qp = 0;
        map_delta_qp = ctx->map.map_delta_qp;
#if ESAO && !ESAO_PH_SYNTAX
        if (ctx->info.sqh.esao_enable_flag)
        {
            enc_eco_esao_param(ctx, bs, &(ctx->info.pic_header));
        }
#endif
#if CCSAO && !CCSAO_PH_SYNTAX
        if (ctx->info.sqh.ccsao_enable_flag)
        {
            enc_eco_ccsao_param(ctx, bs, &ctx->info.pic_header);
        }
#endif
#if UMVE_ENH
        ctx->dataCol = (!ctx->info.sqh.umve_enh_enable_flag || pic_header->slice_type == SLICE_I || pic_header->is_RLpic_flag) ? FALSE : TRUE;
        if (pic_header->slice_type == SLICE_I || pic_header->is_RLpic_flag)
        {
            ctx->needReset = TRUE;
            ctx->lastIPicPOC = pic_header->poc;
        }
        for (u8 idx = 0; idx < UMVE_REFINE_STEP_SEC_SET && ctx->dataCol; idx++)
        {
            ctx->umveOffsetPicCount[idx] = 0;
        }
#endif
        while (1)
        {
            /*set patch idx*/
#if PATCH
        /*set patch idx*/
            if (patch_cur_index != patch->idx)
                enc_set_patch_idx(ctx->map.map_patch_idx, patch, ctx->info.pic_width, ctx->info.pic_height);
            patch_cur_index = patch->idx;
#endif

            if (!ctx->info.shext.fixed_slice_qp_flag)
            {
                int dqp = *map_delta_qp++;
#if CUDQP
                if (com_is_cu_dqp(&ctx->info))
                {
                    assert(dqp == 0);
                }
                else
                {
#endif
                    enc_eco_lcu_delta_qp(bs, dqp, last_lcu_delta_qp);
#if CUDQP
                }
#endif
                last_lcu_delta_qp = dqp;
            }

            if (ctx->info.sqh.sample_adaptive_offset_enable_flag)
            {
                int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
                write_param_sao_one_lcu(ctx, core->y_pel, core->x_pel, ctx->sao_blk_params[lcu_pos]);
            }

#if ESAO_PH_SYNTAX
            if (ctx->info.sqh.esao_enable_flag)
            {
                int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
                for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
                {
                    if (pic_header->pic_esao_on[comp_idx] && pic_header->esao_lcu_enable[comp_idx])
                    {
#if ESAO_ENH
                        enc_eco_esao_lcu_control_flag(GET_SBAC_ENC(bs), bs, ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag[lcu_pos], ctx->info.pic_header.esao_set_num[comp_idx]);
#else
                        enc_eco_esao_lcu_control_flag(GET_SBAC_ENC(bs), bs, ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag[lcu_pos]);
#endif
                    }
                }
            }
#endif

#if CCSAO_PH_SYNTAX
            if (ctx->info.sqh.ccsao_enable_flag)
            {
                int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
                for (int comp = U_C-1; comp < N_C-1; comp++)
                {
                    if (pic_header->pic_ccsao_on[comp] && pic_header->ccsao_lcu_ctrl[comp])
                    {
#if CCSAO_ENHANCEMENT
                        enc_eco_ccsao_lcu_flag(GET_SBAC_ENC(bs), bs, pic_header->pic_ccsao_params[comp].lcu_flag[lcu_pos], ctx->info.pic_header.ccsao_set_num[comp]);
#else               
                        enc_eco_ccsao_lcu_flag(GET_SBAC_ENC(bs), bs, pic_header->pic_ccsao_params[comp].lcu_flag[lcu_pos]);
#endif              
                    }
                }
            }
#endif

            if (ctx->info.sqh.adaptive_leveling_filter_enable_flag)
            {
                int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
                for (int compIdx = Y_C; compIdx < N_C; compIdx++)
                {
                    if (ctx->pic_alf_on[compIdx])
                    {
                        enc_eco_alf_lcu_ctrl(GET_SBAC_ENC(bs), bs, (int)ctx->enc_alf->alf_lcu_enabled[lcu_pos][compIdx]);
                    }
                }
            }

#if NN_FILTER
            if (ctx->info.sqh.nnlf_enable_flag)
            {
                int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
                for (int comp = 0; comp < N_C; comp++)
                {
                    if (ctx->info.pic_header.ph_nnlf_adaptive_flag[comp])
                    {
                        enc_eco_nnlf_lcu_enable_flag(GET_SBAC_ENC(bs), bs, ctx->info.pic_header.nnlf_lcu_enable_flag[comp][lcu_pos], comp);
                        if (ctx->info.pic_header.nnlf_lcu_enable_flag[comp][lcu_pos])
                        {
                            enc_eco_nnlf_lcu_set_index(GET_SBAC_ENC(bs), bs, ctx->info.pic_header.nnlf_lcu_set_index[comp][lcu_pos], comp, ctx->info.sqh.num_of_nnlf);
                        }
                    }
                }
            }
#endif

#if USE_SP
            if (core->x_lcu * ctx->info.max_cuwh == patch->left_pel)
            {
                core->n_pv_num = 0;
            }
#endif
            ret = enc_eco_tree(ctx, core, core->x_pel, core->y_pel, 0, ctx->info.max_cuwh, ctx->info.max_cuwh, 0
                , NO_SPLIT, 0, 0, NO_MODE_CONS, TREE_LC);
            com_assert_rv(ret == COM_OK, ret);
#if USE_SP
            if (ctx->param.evs_ubvs_enable_flag)
            {
                if (ctx->map_cu_data[core->lcu_num].p_SRB)
                {
                    free(ctx->map_cu_data[core->lcu_num].p_SRB);
                }
                if (ctx->map_cu_data[core->lcu_num].pvbuf_reused_flag)
                {
                    free(ctx->map_cu_data[core->lcu_num].pvbuf_reused_flag);
                }
                if (ctx->map_cu_data[core->lcu_num].p_SRB_prev)
                {
                    free(ctx->map_cu_data[core->lcu_num].p_SRB_prev);
                }
                if (ctx->map_cu_data[core->lcu_num].all_comp_flag)
                {
                    free(ctx->map_cu_data[core->lcu_num].all_comp_flag);
                }
                if (ctx->map_cu_data[core->lcu_num].all_comp_pre_flag)
                {
                    free(ctx->map_cu_data[core->lcu_num].all_comp_pre_flag);
                }
                if (ctx->map_cu_data[core->lcu_num].cuS_flag)
                {
                    free(ctx->map_cu_data[core->lcu_num].cuS_flag);
                }
                if (ctx->map_cu_data[core->lcu_num].cuS_pre_flag)
                {
                    free(ctx->map_cu_data[core->lcu_num].cuS_pre_flag);
                }
                if (ctx->map_cu_data[core->lcu_num].pv_x)
                {
                    free(ctx->map_cu_data[core->lcu_num].pv_x);
                }
                if (ctx->map_cu_data[core->lcu_num].pv_x_prev)
                {
                    free(ctx->map_cu_data[core->lcu_num].pv_x_prev);
                }
                if (ctx->map_cu_data[core->lcu_num].pv_y)
                {
                    free(ctx->map_cu_data[core->lcu_num].pv_y);
                }
                if (ctx->map_cu_data[core->lcu_num].pv_y_prev)
                {
                    free(ctx->map_cu_data[core->lcu_num].pv_y_prev);
                }
                if (ctx->map_cu_data[core->lcu_num].srb_u)
                {
                    free(ctx->map_cu_data[core->lcu_num].srb_u);
                }
                if (ctx->map_cu_data[core->lcu_num].srb_v)
                {
                    free(ctx->map_cu_data[core->lcu_num].srb_v);
                }
                if (ctx->map_cu_data[core->lcu_num].m_dpb_reYonly)
                {
                    free(ctx->map_cu_data[core->lcu_num].m_dpb_reYonly);
                }
                if (ctx->map_cu_data[core->lcu_num].m_dpb_reYonly_prev)
                {
                    free(ctx->map_cu_data[core->lcu_num].m_dpb_reYonly_prev);
                }
                if (ctx->map_cu_data[core->lcu_num].m_dpb_idx)
                {
                    free(ctx->map_cu_data[core->lcu_num].m_dpb_idx);
                }
                if (ctx->map_cu_data[core->lcu_num].m_dpb_idx_prev)
                {
                    free(ctx->map_cu_data[core->lcu_num].m_dpb_idx_prev);
                }
            }
#endif
            core->x_lcu++;
#if PATCH
            patch_cur_index = patch->idx;
            if (core->x_lcu >= *(patch->width_in_lcu + patch->x_pat) + patch_cur_lcu_x)
            {
                core->x_lcu = patch_cur_lcu_x;
                core->y_lcu++;
                if (core->y_lcu >= *(patch->height_in_lcu + patch->y_pat) + patch_cur_lcu_y)
                {
                    enc_eco_slice_end_flag(bs, 1);
                    enc_sbac_finish(bs, 0);
                    if (ctx->lcu_cnt > 1)
                    {
                        demulate(bs);
                        /*encode patch end*/
                        assert(COM_BSR_IS_BYTE_ALIGN(bs));
                        ret = enc_eco_send(bs);
                        com_assert_rv(ret == COM_OK, ret);
                        /* Encode patch header */
                        patch_idx = patch->idx + 1;
                        int ph_size = stat->write;
                        com_mset(stat, 0, sizeof(ENC_STAT));
                        stat->write = ph_size + COM_BSW_GET_WRITE_BYTE(bs);
                        com_bsw_init(bs, bs->cur, bs->buftmp, ((int)(bs->end - bs->cur)) + 1, NULL);
                        ret = enc_eco_patch_header(bs, &ctx->info.sqh, pic_header, shext, patch_idx, patch);
                        set_sh(ctx, shext, patch->patch_sao_enable + patch_idx * N_C);
                        // last_lcu_qp = shext->slice_qp; // This initialization is not needed, for delta Qps have been recorded in 'map_delta_qp'.
                        last_lcu_delta_qp = 0;
                        com_assert_rv(ret == COM_OK, ret);
                    }
                    /*update and store map_scu*/
                    en_copy_lcu_scu(map_scu_temp, ctx->map.map_scu, map_refi_temp, ctx->map.map_refi, map_mv_temp, ctx->map.map_mv, map_cu_mode_temp, ctx->map.map_cu_mode, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP
                        ,map_usp_temp, ctx->map.map_usp
#endif
                    );
                    com_mset_x64a(ctx->map.map_scu, 0, sizeof(u32)* ctx->info.f_scu);
#if USE_SP
                    com_mset_x64a(ctx->map.map_usp, 0, sizeof(u8)* ctx->info.f_scu);
#endif
                    if (ctx->lcu_cnt > 1)
                    {
                        /* initialize entropy coder */
                        enc_sbac_init(bs);
                        com_sbac_ctx_init(&(GET_SBAC_ENC(bs)->ctx));
                    }
                    core->x_lcu = *(patch->width_in_lcu + patch->x_pat) + patch_cur_lcu_x;
                    core->y_lcu = patch_cur_lcu_y;
                    patch->x_pat++;
                    if (core->x_lcu >= ctx->info.pic_width_in_lcu)
                    {
                        core->x_lcu = 0;
                        core->y_lcu = *(patch->height_in_lcu + patch->y_pat) + patch_cur_lcu_y;
                        patch->y_pat++;
                        patch->x_pat = 0;
                    }
                    patch->idx++;
                    patch_cur_lcu_x = core->x_lcu;
                    patch_cur_lcu_y = core->y_lcu;
                    core->x_pel = core->x_lcu << ctx->info.log2_max_cuwh;
                    core->y_pel = core->y_lcu << ctx->info.log2_max_cuwh;
                    patch->x_pel = core->x_pel;
                    patch->y_pel = core->y_pel;
                    /*reset the patch boundary*/
                    patch->left_pel = patch->x_pel;
                    patch->up_pel = patch->y_pel;
                    patch->right_pel = patch->x_pel + (*(patch->width_in_lcu + patch->x_pat) << ctx->info.log2_max_cuwh);
                    patch->down_pel = patch->y_pel + (*(patch->height_in_lcu + patch->y_pat) << ctx->info.log2_max_cuwh);
                }
            }
#else
            if (core->x_lcu >= ctx->info.pic_width_in_lcu)
            {
                core->x_lcu = 0;
                core->y_lcu++;
            }
#endif
            core->x_pel = core->x_lcu << ctx->info.log2_max_cuwh;
            core->y_pel = core->y_lcu << ctx->info.log2_max_cuwh;
#if PATCH
            core->lcu_num = core->x_lcu + core->y_lcu*ctx->info.pic_width_in_lcu;
#else
            core->lcu_num++;
#endif
            ctx->lcu_cnt--;
            /* end_of_picture_flag */
            if (ctx->lcu_cnt > 0)
            {
#if PATCH
                if (patch_cur_index == patch->idx)
#endif
                    enc_eco_slice_end_flag(bs, 0);
            }
            else
            {
#if !PATCH
                enc_eco_slice_end_flag(bs, 1);
                enc_sbac_finish(bs, 0);
#endif
                break;
            }
        }

#if UMVE_ENH
        if (ctx->dataCol)
        {
            BOOL umveUsed = FALSE;
            u32  umveOffsetCountTotal = 0;
            for (u8 idx = 0; idx < UMVE_REFINE_STEP_SEC_SET; idx++)
            {
                if (ctx->umveOffsetPicCount[idx] > 0)
                {
                    umveUsed = TRUE;
                    umveOffsetCountTotal += ctx->umveOffsetPicCount[idx];
                }
            }

            if (umveUsed)
            {
                ctx->umveAveOffset = 0.0;
                for (u8 idx = 0; idx < UMVE_REFINE_STEP_SEC_SET; idx++)
                {
                    double umveOffset = 0.25 * (1 << idx);
                    double p = ctx->umveOffsetPicCount[idx] * 1.0 / umveOffsetCountTotal;
                    ctx->umveAveOffset += umveOffset * p;
                }
            }
            else
            {
                ctx->umveAveOffset = -1;
            }
        }
#endif

#if PATCH
        /*get scu from storage*/
        patch->left_pel = 0;
        patch->up_pel = 0;
        patch->right_pel = ctx->info.pic_width;
        patch->down_pel = ctx->info.pic_height;
        en_copy_lcu_scu(ctx->map.map_scu, map_scu_temp, ctx->map.map_refi, map_refi_temp, ctx->map.map_mv, map_mv_temp, ctx->map.map_cu_mode, map_cu_mode_temp, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP
            , ctx->map.map_usp, map_usp_temp
#endif
        );
        com_mfree(patch_sao_enable);
#endif
#if IPPPCRR&&LIB_PIC_UPDATE
    }
#endif
    /* Bit-stream re-writing (END) */
    return COM_OK;
}
#if PATCH
void enc_set_patch_idx(s8 * map_patch_idx, PATCH_INFO * patch, int pic_width, int pic_height)
{
    /*get the boundary*/
    int left_scu = patch->left_pel >> MIN_CU_LOG2;
    int right_scu = patch->right_pel >> MIN_CU_LOG2;
    int down_scu = patch->down_pel >> MIN_CU_LOG2;
    int up_scu = patch->up_pel >> MIN_CU_LOG2;
    int width_scu = pic_width >> MIN_CU_LOG2;
    int height_scu = pic_height >> MIN_CU_LOG2;
    /*set idx*/
    int scu = up_scu*width_scu + left_scu;
    for (int j = up_scu; j < COM_MIN(height_scu, down_scu); j++)
    {
        int temp = scu;
        for (int i = left_scu; i < COM_MIN(width_scu, right_scu); i++)
        {
            map_patch_idx[scu] = (s8)patch->idx;
            scu++;
        }
        scu = temp;
        scu += width_scu;
    }

}
void en_copy_lcu_scu(u32 * scu_temp, u32 * scu_best, s8(*refi_temp)[REFP_NUM], s8(*refi_best)[REFP_NUM], s16(*mv_temp)[REFP_NUM][MV_D], s16(*mv_best)[REFP_NUM][MV_D], u32 *cu_mode_temp, u32 *cu_mode_best, PATCH_INFO * patch, int pic_width, int pic_height
#if USE_SP
    , u8 * usp_temp, u8 * usp_best
#endif
)
{
    /*get the boundary*/
    int left_scu = patch->left_pel >> MIN_CU_LOG2;
    int right_scu = patch->right_pel >> MIN_CU_LOG2;
    int down_scu = patch->down_pel >> MIN_CU_LOG2;
    int up_scu = patch->up_pel >> MIN_CU_LOG2;
    int width_scu = pic_width >> MIN_CU_LOG2;
    int height_scu = pic_height >> MIN_CU_LOG2;
    /*copy*/
    int scu = up_scu*width_scu + left_scu;
    for (int j = up_scu; j < COM_MIN(height_scu, down_scu); j++)
    {
        int temp = scu;
        for (int i = left_scu; i < COM_MIN(width_scu, right_scu); i++)
        {
            scu_temp[scu] = scu_best[scu];
            cu_mode_temp[scu] = cu_mode_best[scu];
#if USE_SP
            usp_temp[scu] = usp_best[scu];
#endif
            for (int m = 0; m < REFP_NUM; m++)
            {
                refi_temp[scu][m] = refi_best[scu][m];
            }
            for (int m = 0; m < REFP_NUM; m++)
            {
                for (int n = 0; n < MV_D; n++)
                {
                    mv_temp[scu][m][n] = mv_best[scu][m][n];
                }
            }
            scu++;
        }
        scu = temp;
        scu += width_scu;
    }
}
#endif
int enc_push_frm(ENC_CTX * ctx, COM_IMGB * imgb)
{
    COM_PIC    * pic;
    ctx->pic_icnt++;
    ctx->pico_idx        = ctx->pic_icnt % ctx->pico_max_cnt;
    ctx->pico            = ctx->pico_buf[ctx->pico_idx];
    ctx->pico->pic_icnt  = ctx->pic_icnt;
    ctx->pico->is_used   = 1;
    pic                  = &ctx->pico->pic;
    PIC_ORG(ctx)        = pic;
    /* set pushed image to current input (original) image */
    com_mset(pic, 0, sizeof(COM_PIC));
    pic->y     = imgb->addr_plane[0];
    pic->u     = imgb->addr_plane[1];
    pic->v     = imgb->addr_plane[2];
    pic->width_luma   = imgb->width[0];
    pic->height_luma   = imgb->height[0];
    pic->width_chroma   = imgb->width[1];
    pic->height_chroma   = imgb->height[1];
    pic->stride_luma   = STRIDE_IMGB2PIC(imgb->stride[0]);
    pic->stride_chroma   = STRIDE_IMGB2PIC(imgb->stride[1]);
    pic->imgb  = imgb;
    imgb->addref(imgb);
#if ALO
    if (ctx->param.alo_enable_type == 1)
    {
        // LD
        if (ctx->param.i_period <= 0 && ctx->pic_icnt % GOP_P == GOP_P - 1)
        {
            updateRefPic(ctx);
        }
        // RA
        else if (ctx->param.i_period > 1 && ctx->pic_icnt % ctx->param.gop_size == ctx->param.gop_size - 1)
        {
            updateRefPic(ctx);
        }
    }
#endif
    return COM_OK;
}


ENC enc_create(ENC_PARAM * param_input, int * err)
{
    ENC_CTX  * ctx;
    int          ret;
#if ENC_DEC_TRACE
    if( fp_trace == NULL )
    {
        fopen_s( &fp_trace, "enc_trace.txt", "w+" );
    }
#endif
    ctx = NULL;
    /* memory allocation for ctx and core structure */
    ctx = ctx_alloc();
    com_assert_gv(ctx != NULL, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    /* set default value for encoding parameter */
    ret = set_init_param(&ctx->param, param_input);
    com_assert_g(ret == COM_OK, ERR);
    /* create intra prediction analyzer */
    ret = pintra_set_complexity(ctx, 0);
    com_assert_g(ret == COM_OK, ERR);
    /* create inter prediction analyzer */
    /* set maximum/minimum value of search range */
    ctx->pinter.min_mv_offset[MV_X] = -MAX_CU_SIZE + 1;
    ctx->pinter.min_mv_offset[MV_Y] = -MAX_CU_SIZE + 1;
    ctx->pinter.max_mv_offset[MV_X] = (s16)ctx->param.pic_width - 1;
    ctx->pinter.max_mv_offset[MV_Y] = (s16)ctx->param.pic_height - 1;
    ret = pinter_set_complexity(ctx, 0);
    com_assert_g(ret == COM_OK, ERR);
#if USE_IBC
    if (ctx->param.use_ibc_flag)
    {
        ctx->pibc.min_clip[MV_X] = -MAX_CU_SIZE + 1;
        ctx->pibc.min_clip[MV_Y] = -MAX_CU_SIZE + 1;
        ctx->pibc.max_clip[MV_X] = ctx->param.pic_width - 1;
        ctx->pibc.max_clip[MV_Y] = ctx->param.pic_height - 1;
        ret = pibc_set_complexity(ctx, 0);
        com_assert_g(ret == COM_OK, ERR);
    }
#endif
#if RDO_DBK
    ctx->pic_dbk = NULL;
#endif
    ctx->pic_sao = NULL;
    ctx->pic_alf_Org = NULL;
    ctx->pic_alf_Rec = NULL;
#if ESAO
    ctx->pic_esao = NULL;
#endif 
#if CCSAO
#if CCSAO_ENHANCEMENT
    ctx->pic_ccsao[0] = NULL;
    ctx->pic_ccsao[1] = NULL;
#else
    ctx->pic_ccsao = NULL;
#endif
#endif
    ret = com_scan_tbl_init();
    com_assert_g(ret == COM_OK, ERR);
#if USE_SP
    ret = com_sp_init();
    com_assert_g(ret == COM_OK, ERR);
#endif
#if FIMC
    com_cntmpm_init(&g_cntMpmInitTable);
#endif

    init_dct_coef();

#if USE_RDOQ
    enc_init_err_scale(ctx->param.bit_depth_internal);
#endif

#if AWP || SAWP
    ctx->awp_init = FALSE;
    enc_init_awp_bufs(ctx);
#endif
#if BGC
    ctx->info.pred_tmp = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
#endif
#if OBMC
    ctx->info.pred_tmp_c[0] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_tmp_c[1] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.subblk_obmc_buf[Y_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.subblk_obmc_buf[U_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.subblk_obmc_buf[V_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_buf_snd[Y_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_buf_snd[U_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_buf_snd[V_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));

    const double PI = 3.14159265358979323846;
    double w = 0;
    int blk_num = 7;

    ctx->info.obmc_weight = (pel ***)calloc(2, sizeof(pel **));
    for (int i = 0; i < 2; i++)
    {
        *(ctx->info.obmc_weight + i) = (pel **)malloc(blk_num * sizeof(pel *));
        for (int j = 0; j < blk_num; j++)
        {
            int blkSize = (1 << j);
            *(*(ctx->info.obmc_weight + i) + j) = (pel *)malloc(blkSize * sizeof(pel));
            for (int s = 0; s < blkSize; s++)
            {
                if (i == 0)
                {
                    w = 0.125 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.875;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
                else
                {
                    w = 0.375 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.625;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
            }
        }
    }
#endif
#if ALO
    ctx->pEncALO = NULL;
#endif
    ret = enc_ready(ctx);
    com_assert_g(ret == COM_OK, ERR);
    /* set default value for ctx */
    ctx->magic = ENC_MAGIC_CODE;
#if PATCH
    //write patch info to ctx
    ctx->patch = com_malloc_fast(sizeof(PATCH_INFO));
    ctx->patch->stable = param_input->patch_stable;
    ctx->patch->cross_patch_loop_filter = param_input->cross_patch_loop_filter;
    ctx->patch->ref_colocated = param_input->patch_ref_colocated;
    if (ctx->patch->stable)
    {
        ctx->patch->uniform = param_input->patch_uniform;
        if (ctx->patch->uniform)
        {
            if (param_input->patch_width_in_lcu <= 0)
            {
                ctx->patch->width = ctx->info.pic_width_in_lcu;
                ctx->patch->columns = 1;
                param_input->patch_column_width[0] = ctx->info.pic_width_in_lcu;
                ctx->patch->width_in_lcu = param_input->patch_column_width;
            }
            else
            {
                ctx->patch->width = param_input->patch_width_in_lcu;
                ctx->patch->columns = ctx->info.pic_width_in_lcu / ctx->patch->width;
                for (int i = 0; i < ctx->patch->columns; i++)
                {
                    param_input->patch_column_width[i] = ctx->patch->width;
                }
                if (ctx->info.pic_width_in_lcu%ctx->patch->width != 0)
                {
                    if (ctx->patch->columns == 0)
                    {
                        param_input->patch_column_width[ctx->patch->columns] = ctx->info.pic_width_in_lcu - ctx->patch->width*ctx->patch->columns;
                        ctx->patch->columns++;
                    }
                    else
                    {
                        param_input->patch_column_width[ctx->patch->columns - 1] = param_input->patch_column_width[ctx->patch->columns - 1] + ctx->info.pic_width_in_lcu - ctx->patch->width*ctx->patch->columns;
                    }
                }
                ctx->patch->width_in_lcu = param_input->patch_column_width;

            }
            if ( param_input->patch_height_in_lcu <=0)
            {
                ctx->patch->height = ctx->info.pic_height_in_lcu;
                ctx->patch->rows = 1;
                param_input->patch_row_height[0] = ctx->info.pic_height_in_lcu;
                ctx->patch->height_in_lcu = param_input->patch_row_height;
            }
            else
            {
                ctx->patch->height = param_input->patch_height_in_lcu;
                ctx->patch->rows = ctx->info.pic_height_in_lcu / ctx->patch->height;
                for (int i = 0; i < ctx->patch->rows; i++)
                {
                    param_input->patch_row_height[i] = ctx->patch->height;
                }
                if (ctx->info.pic_height_in_lcu%ctx->patch->height != 0)
                {
                    if (ctx->patch->rows == 0)
                    {
                        param_input->patch_row_height[ctx->patch->rows] = ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
                        ctx->patch->rows++;
                    }
                    else
                    {
#if PATCH_M4839
                        param_input->patch_row_height[ctx->patch->rows] = ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
                        ctx->patch->rows++;
#else
                        param_input->patch_row_height[ctx->patch->rows - 1] = param_input->patch_row_height[ctx->patch->rows - 1] + ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
#endif
                    }
                }
                ctx->patch->height_in_lcu = param_input->patch_row_height;
            }
            if (ctx->patch->rows * ctx->patch->columns > 128)
            {
                printf("\nError: total number of patches shall be between 1 and 128");
                exit(-1);
            }
        }
    }
#endif
#if UMVE_ENH
    ctx->dataCol = FALSE;
    ctx->needReset = FALSE;
    ctx->lastIPicPOC = -1;
    ctx->umveAveOffset = -1;
#endif
    ctx->id    = (ENC)ctx;
    return (ctx->id);
ERR:
    if(ctx)
    {
        ctx_free(ctx);
    }
    if(err) *err = ret;
    return NULL;
}

#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
void enc_delete(ENC id, int mode)
#else
void enc_delete(ENC id)
#endif
{
    ENC_CTX * ctx;
    ENC_ID_TO_CTX_R(id, ctx);

#if PATCH
    if (ctx->patch)
    {
        free(ctx->patch);
        ctx->patch = NULL;
    }
#endif
#if AWP
    enc_delete_awp_bufs(ctx);
#endif
#if BGC
    if (ctx->info.pred_tmp)
    {
        free( ctx->info.pred_tmp );
        ctx->info.pred_tmp = NULL;
    }
#endif
#if OBMC
    for (int i = 0; i < 2; i++)
    {
        if (ctx->info.pred_tmp_c[1])
        {
            free(ctx->info.pred_tmp_c[i]);
            ctx->info.pred_tmp_c[i] = NULL;
        }
    }
    for (int i = 0; i < N_C; i++)
    {
        if (ctx->info.subblk_obmc_buf[i])
        {
            free(ctx->info.subblk_obmc_buf[i]);
            ctx->info.subblk_obmc_buf[i] = NULL;
        }
        if (ctx->info.pred_buf_snd[i])
        {
            free(ctx->info.pred_buf_snd[i]);
            ctx->info.pred_buf_snd[i] = NULL;
        }
    }
    if (ctx->info.obmc_weight)
    {
        int blk_num = 7;
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < blk_num; j++)
            {
                free(ctx->info.obmc_weight[i][j]);
                ctx->info.obmc_weight[i][j] = NULL;
            }
            free(ctx->info.obmc_weight[i]);
            ctx->info.obmc_weight[i] = NULL;
        }
        free(ctx->info.obmc_weight);
        ctx->info.obmc_weight = NULL;
    }
#endif
    enc_flush(ctx);
    ctx_free(ctx);

#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    if (mode == 1)
    {
#endif
        com_scan_tbl_delete();
#if USE_SP
        com_sp_delete();
#endif

#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    }
#endif
}


static int check_frame_delay(ENC_CTX * ctx)
{
    if(ctx->pic_icnt < ctx->frm_rnum)
    {
        return COM_OK_OUT_NOT_AVAILABLE;
    }
    return COM_OK;
}

static int check_more_frames(ENC_CTX * ctx)
{
    ENC_PICO  * pico;
    int           i;
    if(ctx->param.force_output)
    {
        /* pseudo enc_push() for bumping process ****************/
        ctx->pic_icnt++;
        /**********************************************************/
        for(i=0; i<ctx->pico_max_cnt; i++)
        {
            pico = ctx->pico_buf[i];
            if(pico != NULL)
            {
                if(pico->is_used == 1)
                {
                    return COM_OK;
                }
            }
        }
        return COM_OK_NO_MORE_FRM;
    }
    return COM_OK;
}

int enc_encode(ENC id, COM_BITB * bitb, ENC_STAT * stat)
{
    ENC_CTX * ctx = (ENC_CTX *)id;
    int            ret;
    int            gop_size, pic_cnt;
    /* bumping - check whether input pictures are remaining or not in pico_buf[] */
    if(COM_OK_NO_MORE_FRM == check_more_frames(ctx))
    {
        return COM_OK_NO_MORE_FRM;
    }
    /* store input picture and return if needed */
    if(COM_OK_OUT_NOT_AVAILABLE == check_frame_delay(ctx))
    {
        return COM_OK_OUT_NOT_AVAILABLE;
    }
    /* update BSB */
    bitb->err = 0;
    pic_cnt = ctx->pic_icnt - ctx->frm_rnum;
    gop_size = ctx->param.gop_size;
    ctx->force_slice = ((ctx->pic_ticnt % gop_size >= ctx->pic_ticnt - pic_cnt + 1) && ctx->param.force_output) ? 1 : 0;
    com_assert_rv(bitb->addr && bitb->bsize > 0, COM_ERR_INVALID_ARGUMENT);
    /* initialize variables for a picture encoding */
    ret = enc_pic_prepare(ctx, bitb);
    com_assert_rv(ret == COM_OK, ret);
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    if (ctx->rpm.libvc_data->library_picture_enable_flag)
    {
        if (ctx->is_RLpic_flag)
        {
            if (ctx->rpm.libvc_data->update == 1)
            {
                return RL_UPDATE_TO_LIBPIC;
            }
        }
        if (ctx->rpm.libvc_data->end_of_intra_period == 0 && ctx->rpm.libvc_data->encode_skip == 1) return COM_OK_SKIP;
        if (ctx->rpm.libvc_data->end_of_intra_period == 1 && ctx->rpm.libvc_data->encode_skip == 1)
            ctx->rpm.libvc_data->encode_skip = 1;
        else ctx->rpm.libvc_data->encode_skip = 0;
    }
#endif
#if EXTENSION_USER_DATA
    /* initialize variables for extension data*/
    ret = enc_pic_extension_data_prepare(ctx);
#endif
    /* encode one picture */
    ret = enc_pic(ctx, bitb, stat);
#if LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
    if (ctx->rpm.libvc_data->library_picture_enable_flag)
    if (ctx->rpm.libvc_data->end_of_intra_period == 1 && ctx->rpm.libvc_data->encode_skip == 1) return COM_OK_SKIP;
#endif
    com_assert_rv(ret == COM_OK, ret);
    /* finishing of encoding a picture */
    enc_pic_finish(ctx, bitb, stat);
    com_assert_rv(ret == COM_OK, ret);
#if LIBVC_ON
    /* output libpic to the buffer outside the encoder*/
    if (ctx->info.sqh.library_stream_flag)
    {
        //output to the buffer outside the decoder
        ret = com_picman_out_libpic(PIC_REC(ctx), ctx->info.pic_header.library_picture_index, &ctx->rpm);
    }
#endif
    return COM_OK;
}


void enc_malloc_1d(void** dst, int size)
{
    if (*dst == NULL)
    {
        *dst = com_malloc_fast(size);
        com_mset(*dst, 0, size);
    }
}

void enc_malloc_2d(s8*** dst, int size_1d, int size_2d, int type_size)
{
    int i;
    if (*dst == NULL)
    {
        *dst = com_malloc_fast(size_1d * sizeof(s8*));
        com_mset(*dst, 0, size_1d * sizeof(s8*));
        (*dst)[0] = com_malloc_fast(size_1d * size_2d * type_size);
        com_mset((*dst)[0], 0, size_1d * size_2d * type_size);
        for (i = 1; i < size_1d; i++)
        {
            (*dst)[i] = (*dst)[i - 1] + size_2d * type_size;
        }
    }
}

int enc_create_cu_data(ENC_CU_DATA *cu_data, int cu_width_log2, int cu_height_log2
#if USE_SP
    , u8 sp_flag, u8 evs_ubvs_flag
#endif
)
{
    int i;
    int cuw_scu, cuh_scu;
    int size_8b,size_16b, size_32b, cu_cnt, pixel_cnt;
    cuw_scu = 1 << cu_width_log2;
    cuh_scu = 1 << cu_height_log2;
    size_8b  = cuw_scu * cuh_scu * sizeof(s8);
    size_16b = cuw_scu * cuh_scu * sizeof(s16);
    size_32b = cuw_scu * cuh_scu * sizeof(s32);
    cu_cnt    = cuw_scu * cuh_scu;
    pixel_cnt = cu_cnt << 4;
    enc_malloc_1d((void**)&cu_data->pred_mode, size_8b);
    enc_malloc_1d((void**)&cu_data->ipf_flag, size_8b);
#if IIP
    enc_malloc_1d((void**)&cu_data->iip_flag, size_8b);
#endif
#if INTERPF
    enc_malloc_1d((void**)&cu_data->inter_filter_flag, size_8b); 
#endif
#if IPC
    enc_malloc_1d((void**)&cu_data->ipc_flag, size_8b);
#endif
#if BGC
    enc_malloc_1d((void**)&cu_data->bgc_flag, size_8b);
    enc_malloc_1d((void**)&cu_data->bgc_idx, size_8b);
#endif
    enc_malloc_1d((void**)&cu_data->umve_flag, size_8b);
    enc_malloc_1d((void**)&cu_data->umve_idx, size_8b);
#if AFFINE_UMVE
    enc_malloc_1d((void**)&cu_data->affine_umve_flag, size_8b);
    for (i = 0; i < VER_NUM; i++)
    {
        enc_malloc_1d((void**)&cu_data->affine_umve_idx[i], size_8b);
    }
#endif
#if EXT_AMVR_HMVP
    enc_malloc_1d((void**)&cu_data->mvp_from_hmvp_flag, size_8b);
#endif
#if IBC_BVP
    enc_malloc_1d((void**)&cu_data->cbvp_idx, size_8b);
    enc_malloc_1d((void**)&cu_data->cnt_hbvp_cands, size_8b);
#endif
    enc_malloc_2d((s8***)&cu_data->mpm, 2, cu_cnt, sizeof(u8));
#if SAWP
    enc_malloc_2d((s8***)&cu_data->sawp_mpm, SAWP_MPM_NUM, cu_cnt, sizeof(u8));
#endif // SAWP
    enc_malloc_2d((s8***)&cu_data->ipm, 2, cu_cnt, sizeof(u8));
    enc_malloc_2d((s8***)&cu_data->mpm_ext, 8, cu_cnt, sizeof(u8));
    enc_malloc_2d((s8***)&cu_data->refi, cu_cnt, REFP_NUM, sizeof(u8));
    enc_malloc_1d((void**)&cu_data->mvr_idx, size_8b);
#if IBC_ABVR
    enc_malloc_1d((void**)&cu_data->bvr_idx, size_8b);
#endif
    enc_malloc_1d((void**)&cu_data->skip_idx, size_8b);
#if USE_IBC
    enc_malloc_1d((void**)&cu_data->ibc_flag, size_8b);
#endif
#if USE_SP
    enc_malloc_1d((void**)&cu_data->sp_flag, size_8b);
    enc_malloc_1d((void**)&cu_data->cs2_flag, size_8b);
    if (sp_flag)
    {
        enc_malloc_1d((void**)&cu_data->sub_string_no, size_16b);
        enc_malloc_1d((void**)&cu_data->sp_copy_direction, size_8b);
        enc_malloc_1d((void**)&cu_data->sp_strInfo, (pixel_cnt >> 2) * sizeof(COM_SP_INFO));
        enc_malloc_1d((void**)&cu_data->is_sp_pix_completed, size_8b);
    }
    if (evs_ubvs_flag)
    {
        enc_malloc_1d((void**)&cu_data->evs_copy_direction, size_8b);
        enc_malloc_1d((void**)&cu_data->evs_sub_string_no, cuw_scu * cuh_scu * sizeof(int));
        enc_malloc_1d((void**)&cu_data->unpred_pix_num, cuw_scu * cuh_scu * sizeof(int));
        enc_malloc_1d((void**)&cu_data->equal_val_str_present_flag, size_8b);
        enc_malloc_1d((void**)&cu_data->unpredictable_pix_present_flag, size_8b);
        enc_malloc_1d((void**)&cu_data->pvbuf_size, size_8b);
        enc_malloc_1d((void**)&cu_data->pvbuf_size_prev, size_8b);
        enc_malloc_1d((void**)&cu_data->evs_str_copy_info, pixel_cnt * sizeof(COM_SP_EVS_INFO));
        enc_malloc_1d((void**)&cu_data->unpred_pix_info, pixel_cnt * sizeof(COM_SP_PIX));
    }
#endif
    for (i = 0; i < N_C; i++)
    {
        enc_malloc_1d((void**)&cu_data->num_nz_coef[i], size_32b);
    }
#if TB_SPLIT_EXT
    enc_malloc_1d((void**)&cu_data->pb_part, size_32b);
    enc_malloc_1d((void**)&cu_data->tb_part, size_32b);
#endif
#if USE_SP
    enc_malloc_1d((void**)&cu_data->map_usp, size_8b);
#endif
    enc_malloc_1d((void**)&cu_data->map_scu, size_32b);
    enc_malloc_1d((void**)&cu_data->affine_flag, size_8b);
#if ETMVP
    enc_malloc_1d((void**)&cu_data->etmvp_flag, size_8b);
#endif
#if UNIFIED_HMVP_1
    enc_malloc_1d((void**)&cu_data->mvap_flag, size_8b);
    enc_malloc_1d((void**)&cu_data->sub_tmvp_flag, size_8b);
#endif
#if AWP
    enc_malloc_1d((void**)&cu_data->awp_flag, size_8b);
    enc_malloc_1d((void**)&cu_data->awp_idx0, size_8b);
    enc_malloc_1d((void**)&cu_data->awp_idx1, size_8b);
#endif
#if SAWP
    enc_malloc_1d((void**)&cu_data->sawp_flag, size_8b);
    enc_malloc_1d((void**)&cu_data->sawp_idx0, size_8b);
    enc_malloc_1d((void**)&cu_data->sawp_idx1, size_8b);
#endif
#if AWP_MVR
    enc_malloc_1d((void**)&cu_data->awp_mvr_flag0, size_8b);
    enc_malloc_1d((void**)&cu_data->awp_mvr_idx0, size_8b);
    enc_malloc_1d((void**)&cu_data->awp_mvr_flag1, size_8b);
    enc_malloc_1d((void**)&cu_data->awp_mvr_idx1, size_8b);
#endif
#if SMVD
    enc_malloc_1d( (void**)&cu_data->smvd_flag, size_8b );
#endif
    enc_malloc_1d((void**)&cu_data->map_cu_mode, size_32b);
#if TB_SPLIT_EXT
    enc_malloc_1d((void**)&cu_data->map_pb_tb_part, size_32b);
#endif
#if IST
    enc_malloc_1d((void**)&cu_data->ist_tu_flag, size_8b);
#endif
#if EST
    enc_malloc_1d((void**)&cu_data->est_tu_flag, size_8b);
#endif
#if ST_CHROMA
    enc_malloc_1d((void**)&cu_data->st_chroma_tu_flag, size_8b);
#endif
    enc_malloc_1d((void**)&cu_data->depth, size_8b);
    for (i = 0; i < N_C; i++)
    {
        enc_malloc_1d((void**)&cu_data->coef[i], (pixel_cnt >> (!!(i) * 2)) * sizeof(s16));
        enc_malloc_1d((void**)&cu_data->reco[i], (pixel_cnt >> (!!(i) * 2)) * sizeof(pel));
    }
    return COM_OK;
}

void enc_free_1d(void* dst)
{
    if (dst != NULL)
    {
        com_mfree_fast(dst);
    }
}

void enc_free_2d(void** dst)
{
    if (dst)
    {
        if (dst[0])
        {
            com_mfree_fast(dst[0]);
        }
        com_mfree_fast(dst);
    }
}

int enc_delete_cu_data(ENC_CU_DATA *cu_data
#if USE_SP
    , u8 sp_flag, u8 evs_ubvs_flag
#endif
)
{
    int i;
    enc_free_1d((void*)cu_data->pred_mode);
    enc_free_1d((void*)cu_data->ipf_flag);
#if IIP
    enc_free_1d((void*)cu_data->iip_flag);
#endif
#if INTERPF
    enc_free_1d((void*)cu_data->inter_filter_flag);
#endif
#if IPC
    enc_free_1d((void*)cu_data->ipc_flag);
#endif
#if BGC
    enc_free_1d((void*)cu_data->bgc_flag);
    enc_free_1d((void*)cu_data->bgc_idx);
#endif
    enc_free_1d((void*)cu_data->umve_flag);
    enc_free_1d((void*)cu_data->umve_idx);
#if AFFINE_UMVE
    enc_free_1d((void*)cu_data->affine_umve_flag);
    for (i = 0; i < VER_NUM; i++)
    {
        enc_free_1d((void*)cu_data->affine_umve_idx[i]);
    }
#endif
#if EXT_AMVR_HMVP
    enc_free_1d(cu_data->mvp_from_hmvp_flag);
#endif
#if IBC_BVP
    enc_free_1d(cu_data->cbvp_idx);
    enc_free_1d(cu_data->cnt_hbvp_cands);
#endif
    enc_free_2d((void**)cu_data->mpm);
#if SAWP
    enc_free_2d((void**)cu_data->sawp_mpm);
#endif // SAWP

    enc_free_2d((void**)cu_data->ipm);
    enc_free_2d((void**)cu_data->mpm_ext);
    enc_free_2d((void**)cu_data->refi);
    enc_free_1d(cu_data->mvr_idx);
#if IBC_ABVR
    enc_free_1d(cu_data->bvr_idx);
#endif
    enc_free_1d(cu_data->skip_idx);
#if USE_IBC
    enc_free_1d((void*)cu_data->ibc_flag);
#endif
#if USE_SP
    enc_free_1d((void*)cu_data->sp_flag);
    enc_free_1d((void*)cu_data->cs2_flag);
    if (sp_flag)
    {
        enc_free_1d((void*)cu_data->sub_string_no);
        enc_free_1d((void*)cu_data->sp_copy_direction);
        enc_free_1d((void*)cu_data->sp_strInfo);
        enc_free_1d((void*)cu_data->is_sp_pix_completed);
    }
    if (evs_ubvs_flag)
    {
        enc_free_1d((void*)cu_data->evs_copy_direction);
        enc_free_1d((void*)cu_data->evs_sub_string_no);
        enc_free_1d((void*)cu_data->unpred_pix_num);
        enc_free_1d((void*)cu_data->equal_val_str_present_flag);
        enc_free_1d((void*)cu_data->unpredictable_pix_present_flag);
        enc_free_1d((void*)cu_data->pvbuf_size);
        enc_free_1d((void*)cu_data->pvbuf_size_prev);
        enc_free_1d((void*)cu_data->evs_str_copy_info);
        enc_free_1d((void*)cu_data->unpred_pix_info);
    }
#endif
    for (i = 0; i < N_C; i++)
    {
        enc_free_1d((void*)cu_data->num_nz_coef[i]);
    }
#if TB_SPLIT_EXT
    enc_free_1d((void*)cu_data->pb_part);
    enc_free_1d((void*)cu_data->tb_part);
#endif
#if USE_SP
    enc_free_1d((void*)cu_data->map_usp);
#endif
    enc_free_1d((void*)cu_data->map_scu);
    enc_free_1d((void*)cu_data->affine_flag);
#if ETMVP
    enc_free_1d((void*)cu_data->etmvp_flag);
#endif
#if UNIFIED_HMVP_1
    enc_free_1d((void*)cu_data->mvap_flag);
    enc_free_1d((void*)cu_data->sub_tmvp_flag);
#endif
#if AWP
    enc_free_1d((void*)cu_data->awp_flag);
    enc_free_1d((void*)cu_data->awp_idx0);
    enc_free_1d((void*)cu_data->awp_idx1);
#endif
#if SAWP
    enc_free_1d((void*)cu_data->sawp_flag);
    enc_free_1d((void*)cu_data->sawp_idx0);
    enc_free_1d((void*)cu_data->sawp_idx1);
#endif
#if AWP_MVR
    enc_free_1d((void*)cu_data->awp_mvr_flag0);
    enc_free_1d((void*)cu_data->awp_mvr_idx0);
    enc_free_1d((void*)cu_data->awp_mvr_flag1);
    enc_free_1d((void*)cu_data->awp_mvr_idx1);
#endif
#if SMVD
    enc_free_1d( (void*)cu_data->smvd_flag );
#endif
    enc_free_1d((void*)cu_data->map_cu_mode);
#if TB_SPLIT_EXT
    enc_free_1d((void*)cu_data->map_pb_tb_part);
#endif
#if IST
    enc_free_1d((void*)cu_data->ist_tu_flag);
#endif
#if EST
    enc_free_1d((void*)cu_data->est_tu_flag);
#endif
#if ST_CHROMA
    enc_free_1d((void*)cu_data->st_chroma_tu_flag);
#endif
    enc_free_1d((void*)cu_data->depth);
    for (i = 0; i < N_C; i++)
    {
        enc_free_1d((void*)cu_data->coef[i]);
        enc_free_1d((void*)cu_data->reco[i]);
    }
    return COM_OK;
}
