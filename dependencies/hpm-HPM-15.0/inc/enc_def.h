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

#ifndef _ENC_DEF_H_
#define _ENC_DEF_H_

#include "com_def.h"
#include "enc_bsw.h"
#include "enc_sad.h"

/* support RDOQ */
#define SCALE_BITS               15    /* Inherited from TMuC, pressumably for fractional bit estimates in RDOQ */
#if USE_RDOQ
#define ERR_SCALE_PRECISION_BITS 20
#endif
/* encoder magic code */
#define ENC_MAGIC_CODE         0x45565945 /* EVYE */
#define GOP_P                    8

enum PIC_BUFS
{
    PIC_IDX_ORG = 0,
    PIC_IDX_REC = 1,
    PIC_BUF_NUM = 2,
};

/* motion vector accuracy level for inter-mode decision */
#define ME_LEV_IPEL              1
#define ME_LEV_HPEL              2
#define ME_LEV_QPEL              3

/* maximum inbuf count */
#if RPL_GOP32
#define ENC_MAX_INBUF_CNT      64
#else
#define ENC_MAX_INBUF_CNT      32
#endif

/* maximum cost value */
#define MAX_COST                (1.7e+308)
#if AWP_MVR
#define MAX_U8                  255
#endif

/* virtual frame depth B picture */
#if RPL_GOP32
enum FRM_DEPTH
{
    FRM_DEPTH_0 = 0,
    FRM_DEPTH_1 = 1,
    FRM_DEPTH_2 = 2,
    FRM_DEPTH_3 = 3,
    FRM_DEPTH_4 = 4,
    FRM_DEPTH_5 = 5,
    FRM_DEPTH_6 = 6,
    FRM_DEPTH_MAX
};
#else
#define FRM_DEPTH_0                   0
#define FRM_DEPTH_1                   1
#define FRM_DEPTH_2                   2
#define FRM_DEPTH_3                   3
#define FRM_DEPTH_4                   4
#define FRM_DEPTH_5                   5
#define FRM_DEPTH_MAX                 6
#endif
/* I-slice, P-slice, B-slice + depth + 1 (max for GOP 8 size)*/
#define LIST_NUM                      1

/* instance identifier for encoder */
typedef void  * ENC;

/*****************************************************************************
 * original picture buffer structure
 *****************************************************************************/
typedef struct _ENC_PICO
{
    /* original picture store */
    COM_PIC                pic;
    /* input picture count */
    u32                     pic_icnt;
    /* be used for encoding input */
    u8                      is_used;

    /* address of sub-picture */
    COM_PIC              * spic;
} ENC_PICO;


/*****************************************************************************
 * intra prediction structure
 *****************************************************************************/
typedef struct _ENC_PINTRA
{
    /* temporary prediction buffer */
    pel                 pred[N_C][MAX_CU_DIM];
    pel                 pred_cache[IPD_CNT][MAX_CU_DIM]; // only for luma

    /* reconstruction buffer */
    pel                 rec[N_C][MAX_CU_DIM];

    /* address of original (input) picture buffer */
    pel               * addr_org[N_C];
    /* stride of original (input) picture buffer */
    int                 stride_org[N_C];

    /* address of reconstruction picture buffer */
    pel               * addr_rec_pic[N_C];
    /* stride of reconstruction picture buffer */
    int                 stride_rec[N_C];

    /* QP for luma */
    u8                  qp_y;
    /* QP for chroma */
    u8                  qp_u;
    u8                  qp_v;
#if IIP
    int                 iip_pred[IPD_RDO_CNT];
    int                 iip_rdo_num;
#endif
    int                 slice_type;

    int                 complexity;
    void              * pdata[4];
    int               * ndata[4];

    int                 bit_depth;
} ENC_PINTRA;

/*****************************************************************************
 * inter prediction structure
 *****************************************************************************/

#define MV_RANGE_MIN           0
#define MV_RANGE_MAX           1
#define MV_RANGE_DIM           2

#if INTER_ME_MVLIB
typedef struct _ENC_CORE ENC_CORE;
#endif
#if ENC_ME_IMP
typedef struct _ENC_CTX ENC_CTX;
#endif
typedef struct _ENC_PINTER ENC_PINTER;
struct _ENC_PINTER
{
    int bit_depth;
    /* temporary prediction buffer (only used for ME)*/
    pel  pred_buf[MAX_CU_DIM];
    /* reconstruction buffer */
    pel  rec_buf[N_C][MAX_CU_DIM];

    s16  mvp_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
    s16  mv_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
#if FAST_EXT_AMVR_HMVP
    u8   mvp_flag[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME];
    s16  mvps_uni[2][REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];    // 2 mvps

    int mv_cands_uni_max_size;
    int mv_cands_uni_size[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME];
    s16 mv_cands_uni[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][16][MV_D];
    u32 mv_cands_uni_cost[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][16];
#endif

    u8   curr_mvr;
    int  max_imv[MV_D];

    u8   mvp_from_hmvp_flag;
    u8   imv_valid[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME];
    s16  imv[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];

    int max_search_range;

    CPMV  affine_mvp_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][VER_NUM][MV_D];
    CPMV  affine_mv_scale[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][VER_NUM][MV_D];
    int best_mv_uni[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
    pel  p_error[MAX_CU_DIM];
    int  i_gradient[2][MAX_CU_DIM];
#if SBT || TS_INTER || ST_CHROMA
    pel  resi_cb[N_C][MAX_TR_DIM];
    pel  reco_cb[N_C][MAX_CU_DIM];
    pel  coff_cb[MAX_TR_DIM];
#endif

    s16  org_bi[MAX_CU_DIM];
#if OBMC
    s16  org_obmc[MAX_CU_DIM];
    s16  org_obmc_affine[MAX_CU_DIM];
#endif
    s32  mot_bits[REFP_NUM];

#if DMVR
    pel dmvr_template[MAX_CU_DIM];
    pel dmvr_ref_pred_interpolated[REFP_NUM][(MAX_CU_SIZE + (2*(DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + (2*(DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))];
#endif

    u8   num_refp;
    /* minimum clip value */
    s16  min_mv_offset[MV_D];
    /* maximum clip value */
    s16  max_mv_offset[MV_D];
    /* search range for int-pel */
    s16  search_range_ipel[MV_D];
    /* search range for sub-pel */
    s16  search_range_spel[MV_D];
    s8  (*search_pattern_hpel)[2];
    u8   search_pattern_hpel_cnt;
    s8  (*search_pattern_qpel)[2];
    u8   search_pattern_qpel_cnt;

    /* original (input) picture buffer */
    COM_PIC        *pic_org;
    /* address of original (input) picture buffer */
    pel             *Yuv_org[N_C];
    /* stride of original (input) picture buffer */
    int              stride_org[N_C];
    /* motion vector map */
    s16            (*map_mv)[REFP_NUM][MV_D];
    /* picture width in SCU unit */
    int              pic_width_in_scu;
    /* QP for luma of current encoding CU */
    int               qp_y;
    /* QP for chroma of current encoding CU */
    int               qp_u;
    int               qp_v;
    u32              lambda_mv;
    /* reference pictures */
    COM_REFP      (*refp)[REFP_NUM];
    int              slice_type;
    /* search level for motion estimation */
    int              me_level;
    int              complexity;
    void            *pdata[4];
    int             *ndata[4];
    /* current frame numbser */
    int              ptr;
    /* gop size */
    int              gop_size;
    /* ME function (Full-ME or Fast-ME) */
#if ENC_ME_IMP
#if INTER_ME_MVLIB
    u32(*fn_me)(ENC_CTX *ctx, ENC_PINTER *pi, ENC_CORE *core, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 *refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi);
#else
    u32(*fn_me)(ENC_CTX *ctx, ENC_PINTER *pi, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 *refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi);
#endif
#else
#if INTER_ME_MVLIB
    u32            (*fn_me)(ENC_PINTER *pi, ENC_CORE *core, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 *refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi);
#else
    u32            (*fn_me)(ENC_PINTER *pi, int x, int y, int w, int h, int cu_x, int cu_y, int cu_stride, s8 *refi, int lidx, s16 mvp[MV_D], s16 mv[MV_D], int bi);
#endif
#endif
    /* AFFINE ME function (Gradient-ME) */
#if ASP
    u32            (*fn_affine_me)(COM_INFO* info, ENC_PINTER* pi, int x, int y, int cu_width_log2, int cu_height_log2, s8* refi, int lidx, CPMV mvp[VER_NUM][MV_D], CPMV mv[VER_NUM][MV_D], int bi, int vertex_num, int sub_w, int sub_h);
#else
    u32            (*fn_affine_me)(ENC_PINTER *pi, int x, int y, int cu_width_log2, int cu_height_log2, s8 *refi, int lidx, CPMV mvp[VER_NUM][MV_D], CPMV mv[VER_NUM][MV_D], int bi, int vertex_num, int sub_w, int sub_h);
#endif
};

#if USE_IBC
typedef struct _ENC_PIBC
{
    int  bit_depth;
    int  search_range_x;
    int  search_range_y;
#if IBC_ABVR
    u8   curr_bvr;
    int  max_imv[MV_D];
#endif
    /* reference pictures */
    COM_REFP(*refp)[REFP_NUM];
    u8   mvp_idx;
    /* MV predictor */
    s16  mvp[MV_D];
    s16  mv[MV_D];
    s32  mot_bits;

    /* minimum clip value */
    s16  min_clip[MV_D];
    /* maximum clip value */
    s16  max_clip[MV_D];

    /* original (input) picture buffer */
    COM_PIC        *pic_org;
    /* address of original (input) picture buffer */
    pel            *Yuv_org[N_C];
    /* stride of original (input) picture buffer */
    int            stride_org[N_C];

    /* unfiltered reconstructed picture buffer */
    COM_PIC        *pic_unfiltered_rec;
    /* address of unfiltered reconstructed picture buffer */
    pel            *unfiltered_rec[N_C];
    /* stride of unfiltered reconstructed picture buffer */
    int             s_unfiltered_rec[N_C];

    /* ctu size log2 table */
    s8              ctu_log2_tbl[MAX_CU_SIZE + 1];

    /* picture width in SCU unit */
    u16             pic_width_in_scu;
    /* QP for luma of current encoding CU */
    u8              qp_y;
    /* QP for chroma of current encoding CU */
    u8              qp_u;
    u8              qp_v;
    u32             lambda_mv;

    int             slice_type;
    int             complexity;
    void            *pdata[4];
    int             *ndata[4];
} ENC_PIBC;
#endif

#if USE_SP
typedef struct _ENC_PARENT_INFO
{
    double          p_RDCost;
    double          c_sumRDCost;
    int             p_x;
    int             p_y;
    int             p_width_log2;
    int             p_height_log2;
} ENC_PARENT_INFO;
#endif
/* encoder parameter */
typedef struct _ENC_PARAM
{
#if PHASE_2_PROFILE
    /* profile value */
    int            profile;
#endif
#if CFG_LEVEL_ID
    int            level_id;
#endif
    /* picture size of input sequence (width) */
    int            horizontal_size;
    /* picture size of input sequence (height) */
    int            vertical_size;
    
    /* picture size of pictures in DPB (width) */
    int            pic_width;  // be a multiple of 8 (MINI_SIZE)
    /* picture size of pictures in DPB (height) */
    int            pic_height; // be a multiple of 8 (MINI_SIZE)
    /* qp value for I- and P- slice */
    int            qp;
    /* frame per second */
    int            fps;
    /* I-frame period */
    int            i_period;
    /* force I-frame */
    int            f_ifrm;
    /* picture bit depth*/
    int            bit_depth_input;
    int            bit_depth_internal;
    int            bit_depth_output;
    /* use picture signature embedding */
    int            use_pic_sign;
    int            max_b_frames;
    /* start bumping process if force_output is on */
    int            force_output;
    int            disable_hgop;
    int            gop_size;
#if USE_IBC
    int            use_ibc_flag;
    int            ibc_search_range_x;
    int            ibc_search_range_y;
    int            ibc_hash_search_flag;
    int            ibc_hash_search_max_cand;
    int            ibc_hash_search_range_4smallblk;
    int            ibc_fast_method;
#endif
    int            use_dqp;
    int            frame_qp_add;           /* 10 bits*/
    int            pb_frame_qp_offset;
#if IPCM
    int            ipcm_enable_flag;
#endif
    int            amvr_enable_flag;
#if IBC_ABVR
    int            abvr_enable_flag;
#endif
#if USE_SP
    int            sp_enable_flag;
    int            evs_ubvs_enable_flag;
#endif
#if BIO
    int            bio_enable_flag;
#endif
#if BGC
    int            bgc_enable_flag;
#endif
#if DMVR
    int            dmvr_enable_flag;
#endif
#if INTERPF
    int            interpf_enable_flag;
#endif
#if IPC
    int            ipc_enable_flag;
#endif
    int            affine_enable_flag;
#if ASP
    int            asp_enable_flag;
#endif
#if AFFINE_UMVE
    int            affine_umve_enable_flag;
#endif
    int            smvd_enable_flag;
    int            use_deblock;
#if DBK_SCC
    int            loop_filter_type_enable_flag;
#endif
#if DBR
    int            dbr_enable_flag;
#endif
    int            num_of_hmvp_cand;
#if IBC_BVP
    int            num_of_hbvp_cand;
#endif
#if MIPF
    int            mipf_flag;
#endif
    int            ipf_flag;
#if TSCPM
    int            tscpm_enable_flag;
#if ENHANCE_TSPCM
    int            enhance_tscpm_enable_flag;
#endif
#endif
#if PMC
    int            pmc_enable_flag;
#endif
#if IPF_CHROMA
    int            chroma_ipf_enable_flag;
#endif
#if IIP
    int            iip_enable_flag;
#endif
#if SAWP
    int            sawp_enable_flag;
#endif // SAWP
#if FIMC
    int            fimc_enable_flag;
#endif
    int            umve_enable_flag;
#if UMVE_ENH
    int            umve_enh_enable_flag;
#endif
#if OBMC
    int            obmc_enable_flag;
#endif
#if FAST_LD
    int            fast_ld_me;
#endif
#if AWP
    int            awp_enable_flag;
#endif
#if AWP_MVR
    int            awp_mvr_enable_flag;
#endif
#if ETMVP
    int            etmvp_enable_flag;
#endif
#if SUB_TMVP
    int            sbtmvp_enable_flag;
#endif
#if EXT_AMVR_HMVP
    int            emvr_enable_flag;
#endif
#if DT_PARTITION
    int            dt_intra_enable_flag;
#endif
#if IST
    int            ist_enable_flag;
#endif
#if ISTS
    int            ists_enable_flag;
#endif
#if TS_INTER
    int            ts_inter_enable_flag;
#endif
#if EST
    int            est_enable_flag;
#endif
#if ST_CHROMA
    int            st_chroma_enable_flag;
#endif
#if SRCC
    int            srcc_enable_flag;
#endif
#if CABAC_MULTI_PROB
    int            mcabac_enable_flag;
#endif
#if EIPM
    int            eipm_enable_flag;
#endif
#if MVAP
    int            mvap_enable_flag;
#endif
#if ESAO
    int            esao_enable_flag;
#endif 
#if CCSAO
    int            ccsao_enable_flag;
#endif
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
    int            adaptive_leveling_filter_enhance_flag;
#endif
#if NN_FILTER
    int            num_of_nnlf;
#endif
    int            wq_enable;
    int            seq_wq_mode;
    char           seq_wq_user[2048];
    int            pic_wq_data_idx;
    char           pic_wq_user[2048];
    int            wq_param;
    int            wq_model;
    char           wq_param_detailed[256];
    char           wq_param_undetailed[256];

    int            sample_adaptive_offset_enable_flag;
    int            adaptive_leveling_filter_enable_flag;
    int            secondary_transform_enable_flag;
    u8             position_based_transform_enable_flag;
#if SBT
    int            sbt_enable_flag;
#endif
    u8             library_picture_enable_flag;
    u8             delta_qp_flag;
#if CUDQP
    u8             cu_delta_qp_flag;
    u8             cu_qp_group_size;
#endif
    u8             chroma_format;
    u8             encoding_precision;
#if HLS_RPL
    COM_RPL        rpls_l0[MAX_NUM_RPLS];
    COM_RPL        rpls_l1[MAX_NUM_RPLS];
    int            rpls_l0_cfg_num;
    int            rpls_l1_cfg_num;
#endif
#if PATCH
    int            patch_stable;
    int            cross_patch_loop_filter;
    int            patch_uniform;
    int            patch_ref_colocated;
    int            patch_width_in_lcu;
    int            patch_height_in_lcu;
    int            patch_columns;
    int            patch_rows;
    int            patch_column_width[64];
    int            patch_row_height[128];
#endif
#if LIBVC_ON
    int            qp_offset_libpic;
    int            qp_offset_rlpic;
#endif
    int            sub_sample_ratio;
    int            frames_to_be_encoded;
    u8             ctu_size;
    u8             min_cu_size;
    u8             max_part_ratio;
    u8             max_split_times;
    u8             min_qt_size;
    u8             max_bt_size;
    u8             max_eqt_size;
    u8             max_dt_size;
    int            qp_offset_cb;
    int            qp_offset_cr;
    int            qp_offset_adp;
#if HDR_CHROMA_QP_OFFSET
    int            hdr_chroma_qp_offset;
#endif
    int            bit_depth;
#if ALO
    int            alo_enable_type;
#endif
    int            air_enable_flag;
#if HDR_DISPLAY
    int            colour_description;
    int            colour_primaries;
    int            transfer_charact;
    int            matrix_coeff;
#endif
} ENC_PARAM;

typedef struct _ENC_SBAC
{
    u32            range;
    u32            code;
    int            left_bits;
    u32            stacked_ff;
    u32            pending_byte;
    u32            is_pending_byte;
    COM_SBAC_CTX  ctx;
    u32            bitcounter;
    u8             is_bitcount;
} ENC_SBAC;

typedef struct _ENC_CU_DATA
{
    s8  split_mode[MAX_CU_DEPTH][NUM_BLOCK_SHAPE][MAX_CU_CNT_IN_LCU];
#if TB_SPLIT_EXT
    int* pb_part;
    int* tb_part;
#endif
    u8  *pred_mode;
    u8  **mpm;
#if SAWP
    u8** sawp_mpm;
#endif // SAWP

    u8  **mpm_ext;
    s8  **ipm;

    s8  **refi;
    u8  *mvr_idx;
#if IBC_ABVR
    u8  *bvr_idx;
#endif
#if INTERPF
    u8  *inter_filter_flag;
#endif
#if IPC
    u8  *ipc_flag;
#endif
#if BGC
    u8  *bgc_flag;
    u8  *bgc_idx;
#endif
    u8  *umve_flag;
    u8  *umve_idx;
    u8  *skip_idx;
#if EXT_AMVR_HMVP
    u8  *mvp_from_hmvp_flag;
#endif
#if USE_IBC
    u8  *ibc_flag;
#endif
#if IBC_BVP
    u8  *cbvp_idx;
    s8   *cnt_hbvp_cands;
#endif

#if AFFINE_UMVE
    u8  *affine_umve_flag;
    s8  *affine_umve_idx[VER_NUM];
#endif

#if AWP
    u8  *awp_flag;
    u8  *awp_idx0;
    u8  *awp_idx1;
#endif

#if SAWP
    u8* sawp_flag;
    u8* sawp_idx0;
    u8* sawp_idx1;
#endif

#if AWP_MVR
    u8  *awp_mvr_flag0;
    u8  *awp_mvr_idx0;
    u8  *awp_mvr_flag1;
    u8  *awp_mvr_idx1;
#endif

    s16 mv[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
    s16 mvd[MAX_CU_CNT_IN_LCU][REFP_NUM][MV_D];
    int *num_nz_coef[N_C];
    u32 *map_scu;
    u8  *affine_flag;
#if ETMVP
    u8  *etmvp_flag;
#endif
#if UNIFIED_HMVP_1
    u8  *mvap_flag;
    u8  *sub_tmvp_flag;
#endif
#if SMVD
    u8  *smvd_flag;
#endif
    u32 *map_cu_mode;
#if TB_SPLIT_EXT
    u32 *map_pb_tb_part;
#endif
#if IST
    u8  *ist_tu_flag;
#endif
#if EST
    u8  *est_tu_flag;
#endif
#if ST_CHROMA
    u8  *st_chroma_tu_flag;
#endif
    s8  *depth;
    u8  *ipf_flag;
#if IIP
    u8  *iip_flag;
#endif
    s16 *coef[N_C];
    pel *reco[N_C];
#if USE_SP
    u8  *map_usp;
    u8  *sp_flag;
    u16 *sub_string_no;
    COM_SP_INFO *sp_strInfo;
    u8  *sp_copy_direction;
    u8  *is_sp_pix_completed;
    u8               *cs2_flag;
    u8               *evs_copy_direction;
    int              *evs_sub_string_no;
    COM_SP_EVS_INFO  *evs_str_copy_info;
    int              *unpred_pix_num;
    COM_SP_PIX       *unpred_pix_info;
    u8               *equal_val_str_present_flag;
    u8               *unpredictable_pix_present_flag;
    u8*              pvbuf_size;
    u8*              pvbuf_size_prev;
    s16              *p_SRB;
    u8               *pvbuf_reused_flag;
    s16              *p_SRB_prev;
    u8               *all_comp_flag;
    u8               *all_comp_pre_flag;
    u8               *m_dpb_idx;
    u8               *m_dpb_idx_prev;
    u8               *m_dpb_reYonly;
    u8               *m_dpb_reYonly_prev;
    u8               *cuS_flag;
    u8               *cuS_pre_flag;
    s16              *pv_x;
    s16              *pv_x_prev;
    s16              *pv_y;
    s16              *pv_y_prev;
    s16              *srb_u;
    s16              *srb_v;
#endif
} ENC_CU_DATA;

typedef struct _ENC_BEF_DATA
{
    int    visit;
    int    nosplit;
    int    split;
    int    split_visit;

    double split_cost[MAX_SPLIT_NUM];

    int    mvr_idx_history;
    int    affine_flag_history;
#if IIP
    int    iip_flag_history;
#endif
#if AWP_MVR
    int    awp_flag_history;
    u8     awp_mode_history[AWP_RDO_NUM];
#endif
#if SAWP_SAVE_LOAD
    int    sawp_flag_history;
#endif // SAWP_SAVE_LOAD
#if SAWP_COST_FAST
    double intra_luma_cost;
#endif // SAWP_COST_FAST

    int    smvd_flag_history;
#if EXT_AMVR_HMVP
    int    mvr_hmvp_idx_history;
#endif
    int    amvr_emvr_flag_history;

#if TR_SAVE_LOAD
    u8     num_inter_pred;
    u16    inter_pred_dist[NUM_SL_INTER];
    u8     inter_tb_part[NUM_SL_INTER];  // luma TB part size for inter prediction block
#if SBT_SAVELOAD
    u8     sbt_info_save[NUM_SL_INTER];
#endif
#if IST
    u8     ist_save[NUM_SL_INTER];
#endif
#endif
#if DT_SAVE_LOAD
    u8     best_part_size_intra[2];
    u8     num_intra_history;
#endif
#if  IPC_FAST
    int    lic_flag_history;
#endif
} ENC_BEF_DATA;

#if INTER_ME_MVLIB
typedef struct _BLK_UNI_MV_INFO
{
    s16 uniMvs[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
    int x, y, w, h;
} BLK_UNI_MV_INFO;

typedef struct _ENC_UNIMV_BUF
{
    int visit;
    s16 reused_uni_mv[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D];
} ENC_UNIMV_BUF;

typedef struct _ENC_ME_MVLIB
{
    BLK_UNI_MV_INFO* uni_mv_list;
    int              uni_mv_list_idx;
    int              uni_mv_list_size;
    int              uni_mv_list_max_size;
    ENC_UNIMV_BUF    uni_mv_data[MAX_CU_DEPTH][MAX_CU_DEPTH][MAX_CU_CNT_IN_LCU];
}ENC_ME_MVLIB;
#endif

#if ENC_ME_IMP
typedef struct _AffineMVInfo
{
    CPMV  affMVs[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][VER_NUM][MV_D];
    int x, y, w, h;
} AffineMvInfo;

typedef struct _ENC_AFFINE_UNIMV_BUF
{
    int visit;
    CPMV reused_affine_uni_mv[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][VER_NUM][MV_D];
} ENC_AFFINE_UNIMV_BUF;
#endif

typedef struct _ENC_ALF_VAR
{
#if ALF_IMP 
    ALF_CORR_DATA **alf_corr_arr[ITER_NUM][N_C];
#endif
    ALF_CORR_DATA **alf_corr[N_C];
    ALF_CORR_DATA **alf_non_skipped_corr[N_C];
    ALF_CORR_DATA  *alf_corr_merged[N_C];

    double       *y_merged[NO_VAR_BINS];
    double      **E_merged[NO_VAR_BINS];
#if ALF_SHAPE
    double        y_temp[ALF_MAX_NUM_COEF_SHAPE2];
#else
    double        y_temp[ALF_MAX_NUM_COEF];
#endif
    double        pix_acc_merged[NO_VAR_BINS];
    double      **E_temp;
    ALF_PARAM   **alf_picture_param;
    int          *coeff_num_filter[NO_VAR_BINS];
    int         **filter_coeff_sym;
    int           var_ind_tab[NO_VAR_BINS];
#if ALF_IMP 
    pel         **var_img_arr[ITER_NUM];
#endif
    pel         **var_img;
    BOOL        **alf_lcu_enabled;
    unsigned int  bit_increment;
} ENC_ALF_VAR;

/*****************************************************************************
 * CORE information used for encoding process.
 *
 * The variables in this structure are very often used in encoding process.
 *****************************************************************************/
#if INTER_ME_MVLIB
struct _ENC_CORE
#else
typedef struct _ENC_CORE
#endif
{
    /* mode decision structure */
    COM_MODE       mod_info_best;
    COM_MODE       mod_info_curr;
#if TB_SPLIT_EXT
    COM_MODE       mod_info_save;
    //intra rdo copy the current best info directly into core->mod_info_best; need an internal pb_part for intra
    int            best_pb_part_intra;
    int            best_tb_part_intra;
#endif

    /* coefficient buffer of current CU */
    s16            coef[N_C][MAX_CU_DIM];
    /* CU data for RDO */
    ENC_CU_DATA  cu_data_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    ENC_CU_DATA  cu_data_temp[MAX_CU_DEPTH][MAX_CU_DEPTH];
    /* temporary coefficient buffer */
    s16            ctmp[N_C][MAX_CU_DIM];
#if UMVE_ENH
    pel            predBuf[UMVE_MAX_REFINE_NUM_SEC_SET * UMVE_BASE_NUM + MAX_SKIP_NUM][MAX_CU_DIM];
    double         cu_candCost_list[UMVE_MAX_REFINE_NUM_SEC_SET * UMVE_BASE_NUM + MAX_SKIP_NUM];
    int            cu_cand_list[UMVE_MAX_REFINE_NUM_SEC_SET * UMVE_BASE_NUM + MAX_SKIP_NUM];
#endif
#if OBMC
    BOOL           uni_interpf_ind[UMVE_MAX_REFINE_NUM_SEC_SET * UMVE_BASE_NUM + MAX_SKIP_NUM];
#endif

    /* neighbor pixel buffer for intra prediction */
    pel            nb[N_C][N_REF][MAX_CU_SIZE * 3];
    /* current encoding LCU number */
    int            lcu_num;

    /* QP for luma of current encoding CU */
    int            qp_y;
    /* QP for chroma of current encoding CU */
    int            qp_u;
    int            qp_v;
#if PMC || EPMC
    int            qp_v_pmc;
#endif
    /* X address of current LCU */
    int            x_lcu;
    /* Y address of current LCU */
    int            y_lcu;
    /* X address of current CU in SCU unit */
    int            x_scu;
    /* Y address of current CU in SCU unit */
    int            y_scu;
    /* left pel position of current LCU */
    int            x_pel;
    /* top pel position of current LCU */
    int            y_pel;
    
    /* CU position in current LCU in SCU unit */
    int            cup;
    /* CU depth */
    int            cud;

    /* skip flag for MODE_INTER */
    u8             skip_flag;

    /* split flag for Qt_split_flag */
    u8             split_flag;

    

    /* platform specific data, if needed */
    void          *pf;
    /* bitstream structure for RDO */
    COM_BSW       bs_temp;
    /* SBAC structure for full RDO */
    ENC_SBAC     s_curr_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    ENC_SBAC     s_next_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    ENC_SBAC     s_temp_best;
    ENC_SBAC     s_temp_run;
    ENC_SBAC     s_temp_prev_comp_best;
    ENC_SBAC     s_temp_prev_comp_run;
#if TB_SPLIT_EXT
    ENC_SBAC     s_temp_pb_part_best;
#endif
    ENC_SBAC     s_curr_before_split[MAX_CU_DEPTH][MAX_CU_DEPTH];
    ENC_BEF_DATA bef_data[MAX_CU_DEPTH][MAX_CU_DEPTH][MAX_CU_CNT_IN_LCU];
#if INTER_ME_MVLIB
    ENC_ME_MVLIB s_enc_me_mvlib[MAX_NUM_MVR];
#endif
#if ENC_ME_IMP
    AffineMvInfo* affMVList;
    int           affMVListIdx;
    int           affMVListSize;
    int           affMVListMaxSize;
    ENC_AFFINE_UNIMV_BUF affine_uni_mv_data[MAX_CU_DEPTH][MAX_CU_DEPTH][MAX_CU_CNT_IN_LCU];
#endif
#if TR_SAVE_LOAD
    u8           best_tb_part_hist;
#if SBT_SAVELOAD
    u8           best_sbt_info_hist;
#endif
#if IST
    u8           best_ist_hist;
#endif
#endif
#if TR_EARLY_TERMINATE
    s64          dist_pred_luma;
#endif
#if SBT_FAST
    s64          dist_no_resi[N_C];
#endif

    ENC_SBAC     s_sao_init, s_sao_cur_blk, s_sao_next_blk;
    ENC_SBAC     s_sao_cur_type, s_sao_next_type;
    ENC_SBAC     s_sao_cur_mergetype, s_sao_next_mergetype;
#if ESAO
    ENC_SBAC     s_esao_cur_blk, s_esao_cur_new;
    ENC_SBAC     s_esao_cur_type, s_esao_next_type, s_esao_cur_best;
    ENC_SBAC     s_esao_lcu_open, s_esao_lcu_close, s_esao_lcu_loop;
    ENC_SBAC     s_esao_uvop;
#endif
#if CCSAO
    ENC_SBAC     s_ccsao_init;
    ENC_SBAC     s_ccsao_lcu_on, s_ccsao_lcu_off, s_ccsao_lcu_loop;
#endif

    ENC_SBAC     s_alf_cu_ctr;
    ENC_SBAC     s_alf_initial;

    double         cost_best;
    u32            inter_satd;

    s32            dist_cu;
    s32            dist_cu_best; //dist of the best intra mode (note: only updated in intra coding now)

    // for storing the update-to-date motion list
    COM_MOTION motion_cands[ALLOWED_HMVP_NUM];
#if BGC
    s8 bgc_flag_cands[ALLOWED_HMVP_NUM];
    s8 bgc_idx_cands[ALLOWED_HMVP_NUM];
#endif
    s8 cnt_hmvp_cands;

#if IBC_BVP
    COM_BLOCK_MOTION block_motion_cands[ALLOWED_HBVP_NUM];
    s8 cnt_hbvp_cands;
    s16 bvp_cands[MAX_NUM_BVP][MV_D];
    int cnt_class_cands;
#endif

#if FIMC
    COM_CNTMPM     cntmpm_cands;
#endif

#if EXT_AMVR_HMVP & !FAST_EXT_AMVR_HMVP
    u8    skip_mvps_check;
#endif

#if MVAP
    COM_MOTION    best_cu_mvfield[(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
    COM_MOTION    tmp_cu_mvfield[(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
    COM_MOTION    neighbor_motions[MAX_CU_SIZE + 4];
    s32           valid_mvap_index[ALLOWED_MVAP_NUM];
    s32           valid_mvap_num;
    BOOL          mvap_flag;
    BOOL          best_mvap_flag;
#endif

#if ETMVP
    COM_MOTION    best_etmvp_mvfield[(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
    COM_MOTION    tmp_etmvp_mvfield[(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
#endif

#if USE_SP
    ENC_PARENT_INFO  cu_parent_info;
    COM_MOTION    parent_offset[SP_MAX_SPS_CANDS]; 
    u16           p_offset_num;
    COM_MOTION    brother_offset[SP_MAX_SPS_CANDS];
    u16           b_offset_num;
    s8            n_offset_num;
    COM_MOTION    n_recent_offset[SP_RECENT_CANDS];
    u8            sp_skip_non_scc;
    u8            n_pv_num;
    pel           n_recent_pv[3][MAX_SRB_PRED_SIZE];
    u8            n_pv_num_last;
    pel           n_recent_pv_last[3][MAX_SRB_PRED_SIZE];
    u8            n_recent_all_comp_flag[MAX_SRB_PRED_SIZE];
    u8            n_recent_allcomflag_last[MAX_SRB_PRED_SIZE];
    u8            n_recent_dpb_reYonly[MAX_SRB_PRED_SIZE];
    u8            n_recent_dpb_reYonly_last[MAX_SRB_PRED_SIZE];
    u8            n_recent_dpb_idx[MAX_SRB_PRED_SIZE];
    u8            n_recent_dpb_idx_last[MAX_SRB_PRED_SIZE];
    u8            n_recent_cuS_flag[MAX_SRB_PRED_SIZE];
    u8            n_recent_cuSflag_last[MAX_SRB_PRED_SIZE];
    s16           n_recent_pv_x[MAX_SRB_PRED_SIZE];
    s16           n_recent_pv_x_last[MAX_SRB_PRED_SIZE];
    s16           n_recent_pv_y[MAX_SRB_PRED_SIZE];
    s16           n_recent_pv_y_last[MAX_SRB_PRED_SIZE];
#endif
#if SUB_TMVP
    BOOL          sbTmvp_flag;
    BOOL          best_sbTmvp_flag;
    COM_MOTION    sbTmvp[SBTMVP_NUM];
    COM_MOTION    best_sbTmvp[SBTMVP_NUM];
#endif
#if SCC_CROSSINFO_ULTILIZE
    int ibc_mv[2];
#endif
#if BET_SPLIT_DECSION
    int cur_qt_depth;
    int cur_bet_depth;
#endif
#if INTER_ME_MVLIB
};

void resetUniMvList(ENC_CORE *core, BOOL isLowdelay);
void insertUniMvCands(ENC_ME_MVLIB *mvlib, int x, int y, int w, int h, s16 mvTemp[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][MV_D]);
void savePrevUniMvInfo(ENC_ME_MVLIB *mvlib, int x, int y, int w, int h, BLK_UNI_MV_INFO *tmpUniMvInfo, BOOL *isUniMvInfoSaved);
void addUniMvInfo(ENC_ME_MVLIB *mvlib, BLK_UNI_MV_INFO *tmpUniMVInfo);
#else
} ENC_CORE;
#endif

#if ENC_ME_IMP
void resetAffUniMvList(ENC_CORE *core, BOOL isLD);
void insertAffUniMvCands(ENC_CORE *core, int x, int y, int w, int h, CPMV affMvTemp[REFP_NUM][MAX_NUM_ACTIVE_REF_FRAME][VER_NUM][MV_D]);
void savePrevAffUniMvInfo(ENC_CORE *core, int x, int y, int w, int h, AffineMvInfo *tmpAffUniMvInfo, BOOL *isAffUniMvInfoSaved);
void addAffUniMvInfo(ENC_CORE *core, AffineMvInfo *tmpAffUniMvInfo);
#endif

/******************************************************************************
 * CONTEXT used for encoding process.
 *
 * All have to be stored are in this structure.
 *****************************************************************************/
#if !ENC_ME_IMP
typedef struct _ENC_CTX ENC_CTX;
#endif
struct _ENC_CTX
{
    COM_INFO              info;
    /* address of current input picture, ref_picture  buffer structure */
    ENC_PICO            *pico_buf[ENC_MAX_INBUF_CNT];
    /* address of current input picture buffer structure */
    ENC_PICO            *pico;
    /* index of current input picture buffer in pico_buf[] */
    u32                    pico_idx;
    int                    pico_max_cnt;
    /* magic code */
    u32                    magic;
    /* ENC identifier */
    ENC                  id;
    /* address of core structure */
    ENC_CORE             *core;
    /* address indicating original picture and reconstructed picture */
    COM_PIC              *pic[PIC_BUF_NUM];
    /* reference picture (0: foward, 1: backward) */
    COM_REFP              refp[MAX_NUM_REF_PICS][REFP_NUM];
    /* encoding parameter */
    ENC_PARAM             param;
    /* bitstream structure */
    COM_BSW                bs;
    /* bitstream structure for RDO */
    COM_BSW                bs_temp;

    /* reference picture manager */
    COM_PM                 rpm;

    /* current encoding picture count(This is not PicNum or FrameNum.
    Just count of encoded picture correctly) */
    u32                    pic_cnt;
    /* current picture input count (only update when CTX0) */
    int                    pic_icnt;
    /* total input picture count (only used for bumping process) */
    u32                    pic_ticnt;
    /* remaining pictures is encoded to p or b slice (only used for bumping process) */
    u8                     force_slice;
    /* ignored pictures for force slice count (unavailable pictures cnt in gop,\
    only used for bumping process) */
    int                     force_ignored_cnt;
    /* initial frame return number(delayed input count) due to B picture or Forecast */
    int                    frm_rnum;
    /* current encoding slice number in one picture */
    int                    slice_num;
    /* first mb number of current encoding slice in one picture */
    int                    sl_first_mb;
    /* current slice type */
    u8                     slice_type;
    /* slice depth for current picture */
    u8                     slice_depth;
#if !RPL_GOP32
    /* whether current picture is referred or not */
    u8                     ref_depth;
    /* flag whether current picture is refecened picture or not */
    u8                     slice_ref_flag;
#endif
    /* current picture POC number */
    int                    poc;
    /* maximum CU depth */
    u8                     max_cud;
    ENC_SBAC               sbac_enc;
    /* address of inbufs */
    COM_IMGB              *inbuf[ENC_MAX_INBUF_CNT];
    /* last coded intra picture's presentation temporal reference */
    int                    last_intra_ptr;

    /* total count of remained LCU for encoding one picture. if a picture is
    encoded properly, this value should reach to zero */
    int                    lcu_cnt;

    /* log2 of SCU count in a LCU row */
    u8                     log2_culine;
    /* log2 of SCU count in a LCU (== log2_culine * 2) */
    u8                     log2_cudim;
    /* intra prediction analysis */
    ENC_PINTRA             pintra;
    /* inter prediction analysis */
    ENC_PINTER             pinter;
#if USE_IBC
    /* IBC prediction analysis */
    ENC_PIBC               pibc;
    COM_PIC                *ibc_unfiltered_rec_pic;
#endif
    /* picture buffer allocator */
    PICBUF_ALLOCATOR       pa;
    /* current picture's decoding temporal reference */
    int                    dtr;
    /* current picture's presentation temporal reference */
    int                    ptr;
    /*current picutre's layer id for hierachical structure */
    u8                     temporal_id;

    /* cu data for current LCU */
    ENC_CU_DATA           *map_cu_data;

    COM_MAP                map;
#if SBT_SAVELOAD
    u32                   *sbt_pred_dist;
    u8                    *sbt_info_pred;   //best-mode sbt info
    u8                    *sbt_num_pred;
#endif
#if CHROMA_NOT_SPLIT
    u8                     tree_status;
#endif
#if MODE_CONS
    u8                     cons_pred_mode;
#endif

#if RDO_DBK
    COM_PIC               *pic_dbk;      //one picture that arranges cu pixels and neighboring pixels for deblocking (just to match the interface of deblocking functions)
    s64                    delta_dist;   //delta distortion from filtering (negative values mean distortion reduced)
    s64                    dist_nofilt;  //distortion of not filtered samples
    s64                    dist_filter;  //distortion of filtered samples
#endif

    int                  **edge_filter[LOOPFILTER_DIR_TYPE];

    COM_PIC               *pic_sao;
    SAO_STAT_DATA         ***sao_stat_data; //[SMB][comp][types]
    SAO_BLK_PARAM          **sao_blk_params; //[SMB][comp]
    SAO_BLK_PARAM          **rec_sao_blk_params;//[SMB][comp]
#if ESAO
    COM_PIC               *pic_esao;  //rec buffer
#if ESAO_ENH
    ESAO_BLK_PARAM           init_esao_param[N_C];
    ESAO_STAT_DATA        ***esao_luma_data;
    ESAO_STAT_DATA        ***esao_chroma_data;
    ESAO_BLK_PARAM           temp_esao_param[N_C];
    ESAO_BLK_PARAM           best_esao_param[N_C];
    double                   lambda_esao[N_C];
#else
    ESAO_STAT_DATA         **esao_luma_data;
    ESAO_STAT_DATA         **esao_chroma_data;
#endif
#if !ESAO_PH_SYNTAX
    ESAO_BLK_PARAM           pic_esao_params[N_C];
#endif
    ESAO_FUNC_POINTER        func_esao_block_filter;
#endif
#if CCSAO
#if CCSAO_ENHANCEMENT
    COM_PIC               *pic_ccsao[2];
    CCSAO_STAT_DATA    ****ccsao_chroma_data[N_C-1];
    CCSAO_REFINE_DATA      ccsao_refine_data[N_C-1];
#if ECCSAO
    CCSAO_STAT_DATA    ****ccsao_edge_chroma_data[N_C-1];
#endif
#else
    COM_PIC               *pic_ccsao;
    CCSAO_STAT_DATA     ***ccsao_chroma_data[N_C-1];
    CCSAO_STAT_DATA        ccsao_refine_data[N_C-1];
#endif
    CCSAO_BLK_PARAM        pic_ccsao_params [N_C-1];
    CCSAO_BLK_PARAM        best_ccsao_param [N_C-1];
    CCSAO_BLK_PARAM        temp_ccsao_param [N_C-1];
#if CCSAO_ENHANCEMENT
    CCSAO_BLK_PARAM        init_ccsao_param [N_C-1];
#endif
    double                 lambda_ccsao     [N_C-1];
    CCSAO_FUNC_POINTER     ccsao_func_ptr;
#endif

#if DBR
    DBR_PARAM              dbr_pic_param[6];
    u8                     dbr_pic_poc[6];
#endif

    int                    pic_alf_on[N_C];
    ENC_ALF_VAR             *enc_alf;
    COM_PIC               *pic_alf_Org;
    COM_PIC               *pic_alf_Rec;

    double                 lambda[3];
    double                 sqrt_lambda[3];
    double                 dist_chroma_weight[2];
#if PMC || EPMC
    double                 lambda_v_pmc;
    double                 dist_chroma_weight_v_pmc;
#endif
#if PATCH
    PATCH_INFO             *patch;
#endif
#if LIBVC_ON
    int                    is_RLpic_flag;
#endif
    u8 *wq[2];
#if FIXED_SPLIT
    int                    ctu_idx_in_sequence;
    int                    ctu_idx_in_picture;
    int                    ctu_idx_in_seq_I_slice_ctu;
    int                    ctu_idx_in_seq_B_slice_ctu;
    SPLIT_MODE             split_combination[6];
#endif
#if USE_IBC
    void *ibc_hash_handle;
#endif
#if USE_SP
    void *sp_encoder;
#endif
#if AWP || SAWP
    pel  **** awp_weight0;
    pel  **** awp_weight1;
    pel  **** awp_weight0_scc;
    pel  **** awp_weight1_scc;
    pel  **** awp_bin_weight0;
    pel  **** awp_bin_weight1;
#if FIX_372
    pel  **** awp_weight0_bawp;
    pel  **** awp_weight1_bawp;
    pel  **** awp_bin_weight0_bawp;
    pel  **** awp_bin_weight1_bawp;
#endif
    BOOL ***  awp_larger_area;
    BOOL      awp_init;
#endif
#if AWP
    pel       cand_buff[AWP_MV_LIST_LENGTH][N_C][MAX_CU_DIM];
#endif // AWP
#if UMVE_ENH
    BOOL   dataCol;
    BOOL   needReset;
    s32    lastIPicPOC;
    u32    umveOffsetPicCount[UMVE_REFINE_STEP_SEC_SET];
    double umveAveOffset;
#endif
#if EPMC
    double fcbcr;
#endif
#if AWP_MVR
    pel          candBuffUMVE[AWP_MV_LIST_LENGTH][AWP_MVR_MAX_REFINE_NUM][N_C][MAX_CU_DIM];
#endif
#if CUDQP
    int    cu_delta_qp_lcu_map[256];
    COM_CU_QP_GROUP cu_qp_group;
#endif
#if EXTENSION_USER_DATA
    COM_EXTENSION_DATA ctx_extension_data;
#endif
#if ALO
    void* pEncALO;
#endif
};

/*****************************************************************************
 * API for encoder only
 *****************************************************************************/
ENC enc_create(ENC_PARAM * param, int * err);
#if  LIBVC_ON && IPPPCRR && LIB_PIC_UPDATE
void enc_delete(ENC id,int i);
#else
void enc_delete(ENC id);
#endif
int enc_encode(ENC id, COM_BITB * bitb, ENC_STAT * stat);

int enc_pic_prepare(ENC_CTX * ctx, COM_BITB * bitb);
#if EXTENSION_USER_DATA
int enc_pic_extension_data_prepare(ENC_CTX * ctx);
#endif
int enc_pic_finish(ENC_CTX * ctx, COM_BITB * bitb, ENC_STAT * stat);
int enc_pic(ENC_CTX * ctx, COM_BITB * bitb, ENC_STAT * stat);
int enc_deblock_avs2(ENC_CTX * ctx, COM_PIC * pic
#if DBR
  ,COM_PIC * pic_org
#endif
);
#if DBR
void enc_deblock_frame(ENC_CTX * ctx, COM_INFO *info, COM_MAP *map, COM_PIC * pic, COM_PIC * pic_org, COM_REFP refp[MAX_NUM_REF_PICS][REFP_NUM], int*** edge_filter, double lambda);
#endif
int enc_push_frm(ENC_CTX * ctx, COM_IMGB * img);
int enc_ready(ENC_CTX * ctx);
void enc_flush(ENC_CTX * ctx);
#if AWP || SAWP
void enc_delete_awp_bufs      (ENC_CTX * ctx);
void com_free_4d_Buf          (pel ****array4D,  int wNum,    int hNum,    int Num                );
void com_free_3d_Buf          (pel ***array3D,   int candNum, int compNum                         );
void com_free_3d_Buf_BOOL     (BOOL ***array4D,  int wNum,    int hNum                            );
void enc_init_awp_bufs        (ENC_CTX *ctx                                                       );
void com_malloc_4d_Buf        (pel *****array4D, int wNum,    int hNum,    int Num,    int sizeNum);
void com_malloc_3d_Buf        (pel ****array3D,  int candNum, int compNum, int sizeNum            );
void com_malloc_3d_Buf_BOOL   (BOOL ****array3D, int wNum,    int hNum,    int Num                );
#endif
int enc_picbuf_get_inbuf(ENC_CTX * ctx, COM_IMGB ** img);
#if REPEAT_SEQ_HEADER
int init_seq_header(ENC_CTX * ctx, COM_BITB * bitb);
#else
int enc_seq_header(ENC_CTX * ctx, COM_BITB * bitb, ENC_STAT * stat);
#endif
#if PATCH
void en_copy_lcu_scu(u32 * scu_temp, u32 * scu_best, s8(*refi_temp)[REFP_NUM], s8(*refi_best)[REFP_NUM], s16(*mv_temp)[REFP_NUM][MV_D], s16(*mv_best)[REFP_NUM][MV_D], u32 *cu_mode_temp, u32 *cu_mode_best, PATCH_INFO * patch, int pic_width, int pic_height
#if USE_SP
    , u8 * usp_temp, u8 * usp_best
#endif
);
void enc_set_patch_idx(s8 * map_patch_idx, PATCH_INFO * patch, int pic_width, int pic_height);
#endif
#include "enc_util.h"
#include "enc_eco.h"
#include "enc_mode.h"
#include "enc_tq.h"
#include "enc_pintra.h"
#include "enc_pinter.h"
#if USE_IBC
#include "enc_pibc.h"
#endif
#include "enc_tbl.h"


#endif /* _ENC_DEF_H_ */
