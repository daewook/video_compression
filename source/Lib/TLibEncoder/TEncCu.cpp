/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"

#include <cmath>
#include <algorithm>
using namespace std;

#include <cilk/cilk.h>
//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uiTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight)
{
//  Int i;
  
  m_uhTotalDepth   = uhTotalDepth + 1;
  m_uiMaxWidth = uiMaxWidth;
  m_uiMaxHeight = uiMaxHeight;
//  m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
//  m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];
    
// m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
//  m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
//  m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
//  m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
//  m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
//  m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
//  m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];
  
/*  UInt uiNumPartitions;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );
    UInt uiWidth  = uiMaxWidth  >> i;
    UInt uiHeight = uiMaxHeight >> i;
    
//    m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
//    m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    
//    m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight);
//    m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight);
//    m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight);
    
//    m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight);
//    m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight);
//    m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight);
    
//    m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight);
  }*/
  
  m_bEncodeDQP = false;
#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
  m_LCUPredictionSAD = 0;
  m_addSADDepth      = 0;
  m_temporalSAD      = 0;
#endif

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
  
  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
}

Void TEncCu::destroy()
{
  Int i;
  
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
/*    if(m_ppcBestCU[i])
    {
      m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
    }*/
/*    if(m_ppcTempCU[i])
    {
      m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
    }*/
/*    if(m_ppcPredYuvBest[i])
    {
      m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
    }*/
/*    if(m_ppcResiYuvBest[i])
    {
      m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
    }*/
/*    if(m_ppcRecoYuvBest[i])
    {
      m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
    }*/
/*    if(m_ppcPredYuvTemp[i])
    {
      m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
    }*/
/*    if(m_ppcResiYuvTemp[i])
    {
      m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
    }*/
/*    if(m_ppcRecoYuvTemp[i])
    {
      m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
    }*/
/*    if(m_ppcOrigYuv[i])
    {
      m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
    }*/
  }
/* if(m_ppcBestCU)
  {
    delete [] m_ppcBestCU;
    m_ppcBestCU = NULL;
  }*/
/*  if(m_ppcTempCU)
  {
    delete [] m_ppcTempCU;
    m_ppcTempCU = NULL;
  }*/
  
/*  if(m_ppcPredYuvBest)
  {
    delete [] m_ppcPredYuvBest;
    m_ppcPredYuvBest = NULL;
  }*/
/*  if(m_ppcResiYuvBest)
  {
    delete [] m_ppcResiYuvBest;
    m_ppcResiYuvBest = NULL;
  }*/
/*  if(m_ppcRecoYuvBest)
  {
    delete [] m_ppcRecoYuvBest;
    m_ppcRecoYuvBest = NULL;
  }*/
/*  if(m_ppcPredYuvTemp)
  {
    delete [] m_ppcPredYuvTemp;
    m_ppcPredYuvTemp = NULL;
  }*/
/*  if(m_ppcResiYuvTemp)
  {
    delete [] m_ppcResiYuvTemp;
    m_ppcResiYuvTemp = NULL;
  }*/
/*  if(m_ppcRecoYuvTemp)
  {
    delete [] m_ppcRecoYuvTemp;
    m_ppcRecoYuvTemp = NULL;
  }*/
/*  if(m_ppcOrigYuv)
  {
    delete [] m_ppcOrigYuv;
    m_ppcOrigYuv = NULL;
  }*/
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init( TEncTop* pcEncTop )
{
  m_pcEncCfg           = pcEncTop;
  m_pcPredSearch       = pcEncTop->getPredSearch();
  m_pcTrQuant          = pcEncTop->getTrQuant(); // doesn't need to be parallelized
  m_pcBitCounter       = pcEncTop->getBitCounter(); // doesn't need to be parallelized
  m_pcRdCost           = pcEncTop->getRdCost(); // doesn't need to be parallelized
  
  m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
//  m_pcCavlcCoder       = pcEncTop->getCavlcCoder();
//  m_pcSbacCoder       = pcEncTop->getSbacCoder();
//  m_pcBinCABAC         = pcEncTop->getBinCABAC();
  
  m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();
  
  m_bUseSBACRD        = pcEncTop->getUseSBACRD();
  m_pcRateCtrl        = pcEncTop->getRateCtrl();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncCu::init_predSearch(TEncSearch *search) {
  ((TEncTop *)m_pcEncCfg)->init_TEncSearch(search);
  search->copySearchRange(m_pcPredSearch);
}

/** \param  rpcCU pointer of CU data class
 */
Void TEncCu::compressCU( TEncSlice* slice, TComDataCU*& rpcCU )
{
  // initialize CU data
//  m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
//  m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );

  // initialize NEW_CU data
  DATA data;
  create_DATA(data, 0);
  data.bestCU->initCU(rpcCU->getPic(), rpcCU->getAddr() );
  data.tempCU->initCU(rpcCU->getPic(), rpcCU->getAddr() );

#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
  m_addSADDepth      = 0;
  m_LCUPredictionSAD = 0;
  m_temporalSAD      = 0;
#endif

  // analysis of CU
  TEncSbac *sbac = new TEncSbac;
  TEncBinCABAC *cabac = new TEncBinCABAC;
  sbac->init(cabac);

  // init TEncSearch
  TEncSearch m_cSearch;
  init_predSearch(&m_cSearch);

  // xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0, sbac );
  xCompressCU( &m_cSearch, data, 0, sbac );


#if ADAPTIVE_QP_SELECTION
  if( m_pcEncCfg->getUseAdaptQpSelect() )
  {
    if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
    {
      xLcuCollectARLStats( rpcCU);
    }
  }
#endif

  destroy_DATA(data);
}
/** \param  pcCU  pointer of CU data class
 */
Void TEncCu::encodeCU ( TComDataCU* pcCU )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  // Encode CU data
  xEncodeCU( pcCU, 0, 0 );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Derive small set of test modes for AMP encoder speed-up
 *\param   rpcBestCU
 *\param   eParentPartSize
 *\param   bTestAMP_Hor
 *\param   bTestAMP_Ver
 *\param   bTestMergeAMP_Hor
 *\param   bTestMergeAMP_Ver
 *\returns Void 
*/
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{
  if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
  {
    bTestAMP_Hor = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
  {
    bTestAMP_Ver = true;
  }
  else if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->getMergeFlag(0) == false && rpcBestCU->isSkipped(0) == false )
  {
    bTestAMP_Hor = true;          
    bTestAMP_Ver = true;          
  }

#if AMP_MRG
  //! Utilizing the partition size of parent PU    
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_NONE ) //! if parent is intra
  {
    if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
    {
      bTestMergeAMP_Hor = true;
    }
    else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
      bTestMergeAMP_Ver = true;
    }
  }

  if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->isSkipped(0) == false )
  {
    bTestMergeAMP_Hor = true;          
    bTestMergeAMP_Ver = true;          
  }

  if ( rpcBestCU->getWidth(0) == 64 )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }    
#else
  //! Utilizing the partition size of parent PU        
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  { 
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_2Nx2N )
  { 
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }      
#endif
}
#endif

#ifndef TEST_SBAC
#define TEST_SBAC 0
#endif

Void TEncCu::xCompressCUPart(TEncSearch *search, DATA *data, DATA *subData, TComSlice* pcSlice, UInt uiPartUnitIdx, UInt iQP,
                             UInt uiDepth, UInt uhNextDepth, TEncSbac* pppcRDSbacCoder_curr_best) {

//  printf("part start %d: %p %p\n",(int)uiDepth, data, subData);

  subData->bestCU->initSubCU( data->tempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
  subData->tempCU->initSubCU( data->tempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.

  Bool bInSlice = subData->bestCU->getSCUAddr()+subData->bestCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&subData->bestCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();


//  TEncBinCABAC *cabac1 = new TEncBinCABAC;
  if(bInSlice && ( subData->bestCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( subData->bestCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    if( m_bUseSBACRD )
    {
      if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
      {
        search->getRDSbacCoder()[uhNextDepth][CI_CURR_BEST]->load(search->getRDSbacCoder()[uiDepth][CI_CURR_BEST]);
//        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->init(cabac1);
      }
      else
      {
        search->getRDSbacCoder()[uhNextDepth][CI_CURR_BEST]->load(search->getRDSbacCoder()[uhNextDepth][CI_NEXT_BEST]);
      }
    }

#if TEST_SBAC
   if (pppcRDSbacCoder_curr_best == NULL || !pppcRDSbacCoder_curr_best->compareSbac(search->getRDSbacCoder()[uhNextDepth][CI_CURR_BEST])) {
      printf("coder is not syncrhonized\n");
    }
#endif

#if AMP_ENC_SPEEDUP
    if ( data->bestCU->isIntra(0) )
    {
      xCompressCU( search, *subData, uhNextDepth, NULL, SIZE_NONE);
    }
    else
    {
      xCompressCU( search, *subData, uhNextDepth, NULL, data->bestCU->getPartitionSize(0));
    }
#else
    xCompressCU( search, *subData, uhNextDepth );
#endif

    data->tempCU->copyPartFrom( subData->bestCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
    xCopyYuv2Tmp( *data, *subData, subData->bestCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
  }
  else if (bInSlice)
  {
    subData->bestCU->copyToPic( uhNextDepth );
    data->tempCU->copyPartFrom( subData->bestCU, uiPartUnitIdx, uhNextDepth );
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void TEncCu::create_DATA(DATA& data, UInt depth) {
  UInt uiNumPartitions = 1<<( ( m_uhTotalDepth - depth - 1)<<1 );
  UInt uiWidth = m_uiMaxWidth >> depth;
  UInt uiHeight = m_uiMaxHeight >> depth;

  data.bestCU = new TComDataCU;
  data.tempCU = new TComDataCU;
  data.predYuvBest = new TComYuv;
  data.resiYuvBest = new TComYuv;
  data.recoYuvBest = new TComYuv;
  data.predYuvTemp = new TComYuv;
  data.resiYuvTemp = new TComYuv;
  data.recoYuvTemp = new TComYuv;
  data.origYuv = new TComYuv;

  data.bestCU->create( uiNumPartitions, uiWidth, uiHeight, false, m_uiMaxWidth >> (m_uhTotalDepth - 1)  );
  data.tempCU->create( uiNumPartitions, uiWidth, uiHeight, false, m_uiMaxWidth >> (m_uhTotalDepth - 1) );
  data.predYuvBest->create(uiWidth, uiHeight);
  data.resiYuvBest->create(uiWidth, uiHeight);
  data.recoYuvBest->create(uiWidth, uiHeight);
  data.predYuvTemp->create(uiWidth, uiHeight);
  data.resiYuvTemp->create(uiWidth, uiHeight);
  data.recoYuvTemp->create(uiWidth, uiHeight);
  data.origYuv->create(uiWidth, uiHeight);
}

Void TEncCu::destroy_DATA(DATA& data) {
  return; //TODO make destroy work
  if (data.bestCU) {
    data.bestCU->destroy(); delete data.bestCU; data.bestCU = NULL;
  }
  if (data.tempCU) {
    data.tempCU->destroy(); delete data.tempCU; data.tempCU = NULL;
  }
  if (data.predYuvBest) {
    data.predYuvBest->destroy(); delete data.predYuvBest; data.predYuvBest = NULL;
  }
  if (data.resiYuvBest) {
    data.resiYuvBest->destroy(); delete data.resiYuvBest; data.resiYuvBest = NULL;
  }
  if (data.recoYuvBest) {
    data.recoYuvBest->destroy(); delete data.recoYuvBest; data.recoYuvBest = NULL;
  }
  if (data.predYuvTemp) {
    data.predYuvTemp->destroy(); delete data.predYuvTemp; data.predYuvTemp = NULL;
  }
  if (data.resiYuvTemp) {
    data.resiYuvTemp->destroy(); delete data.resiYuvTemp; data.resiYuvTemp = NULL;
  }
  if (data.recoYuvTemp) {
    data.recoYuvTemp->destroy(); delete data.recoYuvTemp; data.recoYuvTemp = NULL;
  }
  if (data.origYuv) {
    data.origYuv->destroy(); delete data.origYuv; data.origYuv = NULL;
  }
}

/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
*/
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TEncSearch *search, DATA &data, UInt uiDepth, TEncSbac* curr_sbac, PartSize eParentPartSize)
#else
Void TEncCu::xCompressCU( TEncSearch *search, TComDataCU*& rpcBestCU, TComDataCU*& tempCU, UInt uiDepth )
#endif
{
  TComDataCU*& bestCU = data.bestCU;
  TComDataCU*& tempCU = data.tempCU;
/*  TComYuv*& predYuvBest = data.predYuvBest;
  TComYuv*& resiYuvBest = data.resiYuvBest;
  TComYuv*& recoYuvBest = data.recoYuvBest;
  TComYuv*& predYuvTemp = data.predYuvTemp;
  TComYuv*& resiYuvTemp = data.resiYuvTemp;
  TComYuv*& recoYuvTemp = data.recoYuvTemp; */

//  printf("1 %d: %p %p\n",(int)uiDepth, bestCU, tempCU);
  TComPic* pcPic = bestCU->getPic();

  // get Original YUV data from picture
  data.origYuv->copyFromPicYuv( pcPic->getPicYuvOrg(), bestCU->getAddr(), bestCU->getZorderIdxInCU() );

  // variable for Early CU determination
  Bool    bSubBranch = true;

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool earlyDetectionSkipMode = false;

  Bool bBoundary = false;
  UInt uiLPelX   = bestCU->getCUPelX();
  UInt uiRPelX   = uiLPelX + bestCU->getWidth(0)  - 1;
  UInt uiTPelY   = bestCU->getCUPelY();
  UInt uiBPelY   = uiTPelY + bestCU->getHeight(0) - 1;

  Int iBaseQP = xComputeQP( bestCU, uiDepth );
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;
  Int lowestQP = -tempCU->getSlice()->getSPS()->getQpBDOffsetY();

  if( (g_uiMaxCUWidth>>uiDepth) >= tempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -tempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -tempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
    if ( (tempCU->getSlice()->getSPS()->getUseLossless()) && (lowestQP < iMinQP) && tempCU->getSlice()->getPPS()->getUseDQP() )
    {
      isAddLowestQP = true; 
      iMinQP = iMinQP - 1;
    }
  }
  else
  {
    iMinQP = tempCU->getQP(0);
    iMaxQP = tempCU->getQP(0);
  }

#if RATE_CONTROL_LAMBDA_DOMAIN
  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }
#else
  if(m_pcEncCfg->getUseRateCtrl())
  {
    Int qp = m_pcRateCtrl->getUnitQP();
    iMinQP  = Clip3( MIN_QP, MAX_QP, qp);
    iMaxQP  = Clip3( MIN_QP, MAX_QP, qp);
  }
#endif

  // If slice start or slice end is within this cu...
  TComSlice * pcSlice = tempCU->getPic()->getSlice(tempCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>tempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<tempCU->getSCUAddr()+tempCU->getTotalNumPart();
  Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>tempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<tempCU->getSCUAddr()+tempCU->getTotalNumPart());
  Bool bInsidePicture = ( uiRPelX < bestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < bestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
  // We need to split, so don't try these modes.
  if(!bSliceEnd && !bSliceStart && bInsidePicture )
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
      if (isAddLowestQP && (iQP == iMinQP))
      {
        iQP = lowestQP;
      }

      tempCU->initEstData( uiDepth, iQP );

      // do inter modes, SKIP and 2Nx2N
      if( bestCU->getSlice()->getSliceType() != I_SLICE )
      {
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())
        {
          xCheckRDCostInter( search, data, SIZE_2Nx2N );  tempCU->initEstData( uiDepth, iQP );//by Competition for inter_2Nx2N
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( search, data, &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
        tempCU->initEstData( uiDepth, iQP );

        if(!m_pcEncCfg->getUseEarlySkipDetection())
        {
          // 2Nx2N, NxN
          xCheckRDCostInter( search, data, SIZE_2Nx2N );  tempCU->initEstData( uiDepth, iQP );
          if(m_pcEncCfg->getUseCbfFastMode())
          {
            doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
          }
        }
      }

      if (isAddLowestQP && (iQP == lowestQP))
      {
        iQP = iMinQP;
      }
    }

#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
    if ( uiDepth <= m_addSADDepth )
    {
      m_LCUPredictionSAD += m_temporalSAD;
      m_addSADDepth = uiDepth;
    }
#endif

    if(!earlyDetectionSkipMode)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
      {
        if (isAddLowestQP && (iQP == iMinQP))
        {
          iQP = lowestQP;
        }
        tempCU->initEstData( uiDepth, iQP );

        // do inter modes, NxN, 2NxN, and Nx2N
        if( bestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN
          if(!( (bestCU->getWidth(0)==8) && (bestCU->getHeight(0)==8) ))
          {
            if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
            {
              xCheckRDCostInter( search, data, SIZE_NxN   );
              tempCU->initEstData( uiDepth, iQP );
            }
          }

          // 2NxN, Nx2N
          if(doNotBlockPu)
          {
            xCheckRDCostInter( search, data, SIZE_Nx2N  );
            tempCU->initEstData( uiDepth, iQP );
            if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_Nx2N )
            {
              doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( search, data, SIZE_2NxN  );
            tempCU->initEstData( uiDepth, iQP );
            if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_2NxN)
            {
              doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
            }
          }

#if 1
          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
          {
#if AMP_ENC_SPEEDUP        
            Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

            deriveTestModeAMP (bestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
            deriveTestModeAMP (bestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_2NxnU );
                tempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_2NxnD );
                tempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor ) 
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_2NxnU, true );
                tempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_2NxnD, true );
                tempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_nLx2N );
                tempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_nRx2N );
                tempCU->initEstData( uiDepth, iQP );
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_nLx2N, true );
                tempCU->initEstData( uiDepth, iQP );
                if(m_pcEncCfg->getUseCbfFastMode() && bestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = bestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( search, data, SIZE_nRx2N, true );
                tempCU->initEstData( uiDepth, iQP );
              }
            }
#endif

#else
            xCheckRDCostInter( search, data, SIZE_2NxnU );
            tempCU->initEstData( uiDepth, iQP );
            xCheckRDCostInter( search, data, SIZE_2NxnD );
            tempCU->initEstData( uiDepth, iQP );
            xCheckRDCostInter( search, data, SIZE_nLx2N );
            tempCU->initEstData( uiDepth, iQP );

            xCheckRDCostInter( search, data, SIZE_nRx2N );
            tempCU->initEstData( uiDepth, iQP );

#endif
          }    
#endif
        }

        // do normal intra modes
        // speedup for inter frames
        if( bestCU->getSlice()->getSliceType() == I_SLICE || 
          bestCU->getCbf( 0, TEXT_LUMA     ) != 0   ||
          bestCU->getCbf( 0, TEXT_CHROMA_U ) != 0   ||
          bestCU->getCbf( 0, TEXT_CHROMA_V ) != 0     ) // avoid very complex intra if it is unlikely
        {
          xCheckRDCostIntra( search, data, SIZE_2Nx2N );
          tempCU->initEstData( uiDepth, iQP );
          if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
          {
            if( tempCU->getWidth(0) > ( 1 << tempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
            {
              xCheckRDCostIntra( search, data, SIZE_NxN   );
              tempCU->initEstData( uiDepth, iQP );
            }
          }
        }

        // test PCM
        if(pcPic->getSlice(0)->getSPS()->getUsePCM()
          && tempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
          && tempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
        {
          UInt uiRawBits = (2 * g_bitDepthY + g_bitDepthC) * bestCU->getWidth(0) * bestCU->getHeight(0) / 2;
          UInt uiBestBits = bestCU->getTotalBits();
          if((uiBestBits > uiRawBits) || (bestCU->getTotalCost() > search->getPcRdCost()->calcRdCost(uiRawBits, 0)))
          {
            xCheckIntraPCM (search, data);
            tempCU->initEstData( uiDepth, iQP );
          }
        }
        if (isAddLowestQP && (iQP == lowestQP))
        {
          iQP = iMinQP;
        }
      }
    }

    search->getEntropyCoder()->resetBits();
    search->getEntropyCoder()->encodeSplitFlag( bestCU, 0, uiDepth, true );
    bestCU->getTotalBits() += search->getEntropyCoder()->getNumberOfWrittenBits(); // split bits
    if(m_pcEncCfg->getUseSBACRD())
    {
      bestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)search->getEntropyCoder()->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    }
    bestCU->getTotalCost()  = search->getPcRdCost()->calcRdCost( bestCU->getTotalBits(), bestCU->getTotalDistortion() );

    // Early CU determination
    if( m_pcEncCfg->getUseEarlyCU() && bestCU->isSkipped(0) )
    {
      bSubBranch = false;
    }
    else
    {
      bSubBranch = true;
    }
  }
  else if(!(bSliceEnd && bInsidePicture))
  {
    bBoundary = true;
#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
    m_addSADDepth++;
#endif
  }

  // copy orginal YUV samples to PCM buffer
  if( bestCU->isLosslessCoded(0) && (bestCU->getIPCMFlag(0) == false))
  {
    xFillPCMBuffer(bestCU, data.origYuv);
  }
  if( (g_uiMaxCUWidth>>uiDepth) == tempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -tempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -tempCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQP+idQP );
    if ( (tempCU->getSlice()->getSPS()->getUseLossless()) && (lowestQP < iMinQP) && tempCU->getSlice()->getPPS()->getUseDQP() )
    {
      isAddLowestQP = true;
      iMinQP = iMinQP - 1;      
    }
  }
  else if( (g_uiMaxCUWidth>>uiDepth) > tempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    Int iStartQP;
    if( pcPic->getCU( tempCU->getAddr() )->getSliceSegmentStartCU(tempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
    {
      iStartQP = tempCU->getQP(0);
    }
    else
    {
      UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - tempCU->getZorderIdxInCU();
      iStartQP = tempCU->getQP(uiCurSliceStartPartIdx);
    }
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }
#if RATE_CONTROL_LAMBDA_DOMAIN
  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }
#else
  if(m_pcEncCfg->getUseRateCtrl())
  {
    Int qp = m_pcRateCtrl->getUnitQP();
    iMinQP  = Clip3( MIN_QP, MAX_QP, qp);
    iMaxQP  = Clip3( MIN_QP, MAX_QP, qp);
  }
#endif

  for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
  {
    if (isAddLowestQP && (iQP == iMinQP))
    {
      iQP = lowestQP;
    }
    tempCU->initEstData( uiDepth, iQP );

    // further split
    if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      UChar       uhNextDepth         = uiDepth+1;
//      TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
//      TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
/////////////////////////////////////////////////////////////////////////////
      DATA subData, subData2;

      create_DATA(subData, uhNextDepth);
      create_DATA(subData2, uhNextDepth);
/*      TComDataCU pcSubBestPartCU;
      TComDataCU pcSubTempPartCU;

      create_TComDataCU(pcSubBestPartCU, uhNextDepth);
      create_TComDataCU(pcSubTempPartCU, uhNextDepth);
  
      TComDataCU pcSubBestPartCU2;
      TComDataCU pcSubTempPartCU2;

      create_TComDataCU(pcSubBestPartCU2, uhNextDepth);
      create_TComDataCU(pcSubTempPartCU2, uhNextDepth);*/

/////////////////////////////////////////////////////////////////////////////////


      TEncSbac *sbac1 = new TEncSbac;
      TEncBinCABAC *cabac1 = new TEncBinCABAC;
      sbac1->init(cabac1);

      if (curr_sbac != NULL) sbac1->load(curr_sbac);
//      m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);


      xCompressCUPart(search, &data, &subData, pcSlice, 0, iQP, uiDepth, uhNextDepth, sbac1);
      // init TEncSearch
      TEncSearch search2;
      init_predSearch(&search2);
      search2.copySearch(search, pcSlice);

      //printf("depth: %d new spawn\n", uiDepth);
      xCompressCUPart(search, &data, &subData, pcSlice, 1, iQP, uiDepth, uhNextDepth, sbac1);
      xCompressCUPart(&search2, &data, &subData2, pcSlice, 2, iQP, uiDepth, uhNextDepth, sbac1);
      // cilk_sync;
      //printf("depth: %d spawn synced\n", uiDepth);
//      search->copySearch(&search2, pcSlice);
//      cilk_sync;
      xCompressCUPart(search, &data, &subData, pcSlice, 3, iQP, uiDepth, uhNextDepth, sbac1);
      // done

//      destroy_DATA(subData);
//      destroy_DATA(subData2);
      delete sbac1;
      delete cabac1;
      sbac1 = NULL;
      cabac1 = NULL;

      if( !bBoundary )
      {
        search->getEntropyCoder()->resetBits();
        search->getEntropyCoder()->encodeSplitFlag( tempCU, 0, uiDepth, true );

        tempCU->getTotalBits() += search->getEntropyCoder()->getNumberOfWrittenBits(); // split bits
        if(m_pcEncCfg->getUseSBACRD())
        {
          tempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)search->getEntropyCoder()->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
        }
      }
      tempCU->getTotalCost()  = search->getPcRdCost()->calcRdCost( tempCU->getTotalBits(), tempCU->getTotalDistortion() );

      if( (g_uiMaxCUWidth>>uiDepth) == tempCU->getSlice()->getPPS()->getMinCuDQPSize() && tempCU->getSlice()->getPPS()->getUseDQP())
      {
        Bool hasResidual = false;
        for( UInt uiBlkIdx = 0; uiBlkIdx < tempCU->getTotalNumPart(); uiBlkIdx ++)
        {
          if( ( pcPic->getCU( tempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+tempCU->getZorderIdxInCU()) == tempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) && 
              ( tempCU->getCbf( uiBlkIdx, TEXT_LUMA ) || tempCU->getCbf( uiBlkIdx, TEXT_CHROMA_U ) || tempCU->getCbf( uiBlkIdx, TEXT_CHROMA_V ) ) )
          {
            hasResidual = true;
            break;
          }
        }

        UInt uiTargetPartIdx;
        if ( pcPic->getCU( tempCU->getAddr() )->getSliceSegmentStartCU(tempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
        {
          uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - tempCU->getZorderIdxInCU();
        }
        else
        {
          uiTargetPartIdx = 0;
        }
        if ( hasResidual )
        {
#if !RDO_WITHOUT_DQP_BITS
          search->getEntropyCoder()->resetBits();
          search->getEntropyCoder()->encodeQP( tempCU, uiTargetPartIdx, false );
          tempCU->getTotalBits() += search->getEntropyCoder()->getNumberOfWrittenBits(); // dQP bits
          if(m_pcEncCfg->getUseSBACRD())
          {
            tempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)search->getEntropyCoder()->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          }
          tempCU->getTotalCost()  = search->getPcRdCost()->calcRdCost( tempCU->getTotalBits(), tempCU->getTotalDistortion() );
#endif

          Bool foundNonZeroCbf = false;
          tempCU->setQPSubCUs( tempCU->getRefQP( uiTargetPartIdx ), tempCU, 0, uiDepth, foundNonZeroCbf );
          assert( foundNonZeroCbf );
        }
        else
        {
          tempCU->setQPSubParts( tempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
        }
      }

      if( m_bUseSBACRD )
      {
        search->getRDSbacCoder()[uhNextDepth][CI_NEXT_BEST]->store(search->getRDSbacCoder()[uiDepth][CI_TEMP_BEST]);
      }
      Bool isEndOfSlice        = bestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
                                 && (bestCU->getTotalBits()>bestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = bestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
                                 && (bestCU->getTotalBits()>bestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        bestCU->getTotalCost()=tempCU->getTotalCost()+1;
      }
      xCheckBestMode( search, data, uiDepth);                                  // RD compare current larger prediction
    }                                                                                  // with sub partitioned prediction.
    if (isAddLowestQP && (iQP == lowestQP))
    {
      iQP = iMinQP;
    }
  }

  bestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.

  xCopyYuv2Pic( bestCU->getPic(), bestCU->getAddr(), bestCU->getZorderIdxInCU(), uiDepth, uiDepth, bestCU, uiLPelX, uiTPelY, data );   // Copy Yuv data to picture Yuv

//  printf("9 %d: %p %p\n",(int)uiDepth, bestCU, tempCU);

  if( bBoundary ||(bSliceEnd && bInsidePicture))
  {
    return;
  }

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  assert( bestCU->getPartitionSize ( 0 ) != SIZE_NONE  );
  assert( bestCU->getPredictionMode( 0 ) != MODE_NONE  );
  assert( bestCU->getTotalCost     (   ) != MAX_DOUBLE );
}

/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::finishCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

  //Calculate end address
  UInt uiCUAddr = pcCU->getSCUAddr()+uiAbsPartIdx;

  UInt uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
  UInt uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
  UInt uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
  UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
  while(uiPosX>=uiWidth||uiPosY>=uiHeight)
  {
    uiInternalAddress--;
    uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
    uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  }
  uiInternalAddress++;
  if(uiInternalAddress==pcCU->getPic()->getNumPartInCU())
  {
    uiInternalAddress = 0;
    uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
  }
  UInt uiRealEndAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);

  // Encode slice finish
  Bool bTerminateSlice = false;
  if (uiCUAddr+(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)) == uiRealEndAddress)
  {
    bTerminateSlice = true;
  }
  UInt uiGranularityWidth = g_uiMaxCUWidth;
  uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Bool granularityBoundary=((uiPosX+pcCU->getWidth(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosX+pcCU->getWidth(uiAbsPartIdx)==uiWidth))
    &&((uiPosY+pcCU->getHeight(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosY+pcCU->getHeight(uiAbsPartIdx)==uiHeight));
  
  if(granularityBoundary)
  {
    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    if (!bTerminateSlice)
      m_pcEntropyCoder->encodeTerminatingBit( bTerminateSlice ? 1 : 0 );
  }
  
  Int numberOfWrittenBits = 0;
  if (m_pcBitCounter)
  {
    numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  }
  
  // Calculate slice end IF this CU puts us over slice bit size.
  UInt iGranularitySize = pcCU->getPic()->getNumPartInCU();
  Int iGranularityEnd = ((pcCU->getSCUAddr()+uiAbsPartIdx)/iGranularitySize)*iGranularitySize;
  if(iGranularityEnd<=pcSlice->getSliceSegmentCurStartCUAddr()) 
  {
    iGranularityEnd+=max(iGranularitySize,(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)));
  }
  // Set slice end parameter
  if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceBits()+numberOfWrittenBits>pcSlice->getSliceArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    pcSlice->setSliceCurEndCUAddr(iGranularityEnd);
    return;
  }
  // Set dependent slice end parameter
  if(pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceSegmentBits()+numberOfWrittenBits > pcSlice->getSliceSegmentArgument()<<3) 
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    return;
  }
  if(granularityBoundary)
  {
    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);
    if (m_pcBitCounter)
    {
      m_pcEntropyCoder->resetBits();      
    }
  }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP( TComDataCU* pcCU, UInt uiDepth )
{
  Int iBaseQp = pcCU->getSlice()->getSliceQp();
  Int iQpOffset = 0;
  if ( m_pcEncCfg->getUseAdaptiveQP() )
  {
    TEncPic* pcEPic = dynamic_cast<TEncPic*>( pcCU->getPic() );
    UInt uiAQDepth = min( uiDepth, pcEPic->getMaxAQDepth()-1 );
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer( uiAQDepth );
    UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
    UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
    UInt uiAQUStride = pcAQLayer->getAQPartStride();
    TEncQPAdaptationUnit* acAQU = pcAQLayer->getQPAdaptationUnit();

    Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange()/6.0);
    Double dAvgAct = pcAQLayer->getAvgActivity();
    Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
    Double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
    Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
    iQpOffset = Int(floor( dQpOffset + 0.49999 ));
  }
  return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffsetY(), MAX_QP, iBaseQp+iQpOffset );
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth 
 * \returns Void
 */
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  // If slice start is within this cu...
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcPic->getNumPartInCU() >> (uiDepth<<1) );
  // We need to split, so don't try these modes.
  if(!bSliceStart&&( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    m_pcEntropyCoder->encodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
  }
  else
  {
    bBoundary = true;
  }
  
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
    {
      setdQPFlag(true);
    }
    int ref[4];
    ref[0] = uiAbsPartIdx;
    for (int i = 1; i < 4; i++) {
      ref[i] = ref[i - 1] + uiQNumParts;
    }
    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++) 
    {
      //uiAbsPartIdx+=uiQNumParts;
      int uiAbsPartIdx_ = ref[uiPartUnitIdx];
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx_] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx_] ];
      Bool bInSlice = pcCU->getSCUAddr()+uiAbsPartIdx_+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx_<pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xEncodeCU( pcCU, uiAbsPartIdx_, uiDepth+1 );
      }
    }
    return;
  }
  
  if( (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
  {
    setdQPFlag(true);
  }
  if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );
  }
  if( !pcCU->getSlice()->isIntra() )
  {
    m_pcEntropyCoder->encodeSkipFlag( pcCU, uiAbsPartIdx );
  }
  
  if( pcCU->isSkipped( uiAbsPartIdx ) )
  {
    m_pcEntropyCoder->encodeMergeIndex( pcCU, uiAbsPartIdx );
    finishCU(pcCU,uiAbsPartIdx,uiDepth);
    return;
  }
  m_pcEntropyCoder->encodePredMode( pcCU, uiAbsPartIdx );
  
  m_pcEntropyCoder->encodePartSize( pcCU, uiAbsPartIdx, uiDepth );
  
  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
  {
    m_pcEntropyCoder->encodeIPCMInfo( pcCU, uiAbsPartIdx );

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
      // Encode slice finish
      finishCU(pcCU,uiAbsPartIdx,uiDepth);
      return;
    }
  }

  // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
  m_pcEntropyCoder->encodePredInfo( pcCU, uiAbsPartIdx );
  
  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  m_pcEntropyCoder->encodeCoeff( pcCU, uiAbsPartIdx, uiDepth, pcCU->getWidth (uiAbsPartIdx), pcCU->getHeight(uiAbsPartIdx), bCodeDQP );
  setdQPFlag( bCodeDQP );

  // --- write terminating bit ---
  finishCU(pcCU,uiAbsPartIdx,uiDepth);
}

#if RATE_CONTROL_INTRA
Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg) 
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] ;
    diff[k+1] = piOrg[1] ;
    diff[k+2] = piOrg[2] ;
    diff[k+3] = piOrg[3] ;
    diff[k+4] = piOrg[4] ;
    diff[k+5] = piOrg[5] ;
    diff[k+6] = piOrg[6] ;
    diff[k+7] = piOrg[7] ;
 
    piOrg += iStrideOrg;
  }
  
  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];
    
    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];
    
    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }
  
  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];
    
    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];
    
    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }
  
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad =(iSumHad+2)>>2;
  return(iSumHad);
}

Int  TEncCu::updateLCUDataISlice(TComDataCU* pcCU, Int LCUIdx, Int width, Int height)
{
  Int  xBl, yBl; 
  const Int iBlkSize = 8;

  Pel* pOrgInit   = pcCU->getPic()->getPicYuvOrg()->getLumaAddr(pcCU->getAddr(), 0);
  Int  iStrideOrig = pcCU->getPic()->getPicYuvOrg()->getStride();
  Pel  *pOrg;

  Int iSumHad = 0;
  for ( yBl=0; (yBl+iBlkSize)<=height; yBl+= iBlkSize)
  {
    for ( xBl=0; (xBl+iBlkSize)<=width; xBl+= iBlkSize)
    {
      pOrg = pOrgInit + iStrideOrig*yBl + xBl; 
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}
#endif

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2N( TEncSearch *search, DATA &data, Bool *earlyDetectionSkipMode )
{
  assert( data.tempCU->getSlice()->getSliceType() != I_SLICE );
  TComMvField  cMvFieldNeighbours[ 2 * MRG_MAX_NUM_CANDS ]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;

  for( UInt ui = 0; ui < data.tempCU->getSlice()->getMaxNumMergeCand(); ++ui )
  {
    uhInterDirNeighbours[ui] = 0;
  }
  UChar uhDepth = data.tempCU->getDepth( 0 );
  data.tempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
  data.tempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uhDepth );
  data.tempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
  
  Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
  for( UInt ui = 0; ui < numValidMergeCand; ++ui )
  {
    mergeCandBuffer[ui] = 0;
  }

  Bool bestIsSkip = false;

  UInt iteration;
  if ( data.tempCU->isLosslessCoded(0))
  {
    iteration = 1;
  }
  else 
  {
    iteration = 2;
  }

  for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )
  {
    for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
    {
      if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))
      {
        if( !(bestIsSkip && uiNoResidual == 0) )
        {
          // set MC parameters
          data.tempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
          data.tempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(),     0, uhDepth );
          data.tempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
          data.tempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
          data.tempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
          data.tempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
          data.tempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to data.tempCU level
          data.tempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to data.tempCU level
          
          // do MC
          search->motionCompensation ( data.tempCU, data.predYuvTemp );
          // estimate residual and encode everything
          search->encodeResAndCalcRdInterCU( data.tempCU,
                                                     data.origYuv,
                                                    data.predYuvTemp,
                                                    data.resiYuvTemp,
                                                    data.resiYuvBest,
                                                    data.recoYuvTemp,
                                                    (uiNoResidual? true:false));
          
          
          if ( uiNoResidual == 0 && data.tempCU->getQtRootCbf(0) == 0 )
          {
            // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
            mergeCandBuffer[uiMergeCand] = 1;
          }
          
          data.tempCU->setSkipFlagSubParts( data.tempCU->getQtRootCbf(0) == 0, 0, uhDepth );
          Int orgQP = data.tempCU->getQP( 0 );
          xCheckDQP( search, data.tempCU );
          xCheckBestMode(search, data, uhDepth);
          data.tempCU->initEstData( uhDepth, orgQP );
          
          if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
          {
            bestIsSkip = data.bestCU->getQtRootCbf(0) == 0;
          }
        }
      }
    }
    
    if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      if(data.bestCU->getQtRootCbf( 0 ) == 0)
      {
        if( data.bestCU->getMergeFlag( 0 ))
        {
          *earlyDetectionSkipMode = true;
        }
        else
        {
          Int absoulte_MV=0;
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( data.bestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              TComCUMvField* pcCUMvField = data.bestCU->getCUMvField(RefPicList( uiRefListIdx ));
              Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
              Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
              absoulte_MV+=iHor+iVer;
            }
          }
          
          if(absoulte_MV == 0)
          {
            *earlyDetectionSkipMode = true;
          }
        }
      }
    }
  }
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInter( TEncSearch *search, DATA &data, PartSize ePartSize, Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInter( TEncSearch *search, DATA &data, PartSize ePartSize )
#endif
{
  UChar uhDepth = data.tempCU->getDepth( 0 );
  
  data.tempCU->setDepthSubParts( uhDepth, 0 );
  
  data.tempCU->setSkipFlagSubParts( false, 0, uhDepth );

  data.tempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
  data.tempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
  data.tempCU->setCUTransquantBypassSubParts  ( m_pcEncCfg->getCUTransquantBypassFlagValue(),      0, uhDepth );
  
#if AMP_MRG
  data.tempCU->setMergeAMP (true);
  search->predInterSearch ( data.tempCU, data.origYuv, data.predYuvTemp, data.resiYuvTemp, data.recoYuvTemp, false, bUseMRG );
#else  
  search->predInterSearch ( data.tempCU, data.origYuv, data.predYuvTemp, data.resiYuvTemp, data.recoYuvTemp );
#endif

#if AMP_MRG
  if ( !data.tempCU->getMergeAMP() )
  {
    return;
  }
#endif

#if RATE_CONTROL_LAMBDA_DOMAIN && !M0036_RC_IMPROVEMENT
  if ( m_pcEncCfg->getUseRateCtrl() && m_pcEncCfg->getLCULevelRC() && ePartSize == SIZE_2Nx2N && uhDepth <= m_addSADDepth )
  {
    UInt SAD = search->getPcRdCost()->getSADPart( g_bitDepthY, data.predYuvTemp->getLumaAddr(), data.predYuvTemp->getStride(),
      data.origYuv->getLumaAddr(), data.origYuv->getStride(),
      data.tempCU->getWidth(0), data.tempCU->getHeight(0) );
    m_temporalSAD = (Int)SAD;
  }
#endif

  search->encodeResAndCalcRdInterCU( data.tempCU, data.origYuv, data.predYuvTemp, data.resiYuvTemp, data.resiYuvBest, data.recoYuvTemp, false );
  data.tempCU->getTotalCost()  = search->getPcRdCost()->calcRdCost( data.tempCU->getTotalBits(), data.tempCU->getTotalDistortion() );

  xCheckDQP( search, data.tempCU );
  xCheckBestMode(search, data, uhDepth);
}

Void TEncCu::xCheckRDCostIntra( TEncSearch *search, DATA &data, PartSize eSize )
{
  UInt uiDepth = data.tempCU->getDepth( 0 );
  
  data.tempCU->setSkipFlagSubParts( false, 0, uiDepth );

  data.tempCU->setPartSizeSubParts( eSize, 0, uiDepth );
  data.tempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  data.tempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uiDepth );
  
  Bool bSeparateLumaChroma = true; // choose estimation mode
  UInt uiPreCalcDistC      = 0;
  if( !bSeparateLumaChroma )
  {
    search->preestChromaPredMode( data.tempCU, data.origYuv, data.predYuvTemp );
  }
  search  ->estIntraPredQT      ( data.tempCU, data.origYuv, data.predYuvTemp, data.resiYuvTemp, data.recoYuvTemp, uiPreCalcDistC, bSeparateLumaChroma );

  data.recoYuvTemp->copyToPicLuma(data.tempCU->getPic()->getPicYuvRec(), data.tempCU->getAddr(), data.tempCU->getZorderIdxInCU() );
  
  search  ->estIntraPredChromaQT( data.tempCU, data.origYuv, data.predYuvTemp, data.resiYuvTemp, data.recoYuvTemp, uiPreCalcDistC );
  
  search->getEntropyCoder()->resetBits();
  if ( data.tempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    search->getEntropyCoder()->encodeCUTransquantBypassFlag( data.tempCU, 0,          true );
  }
  search->getEntropyCoder()->encodeSkipFlag ( data.tempCU, 0,          true );
  search->getEntropyCoder()->encodePredMode( data.tempCU, 0,          true );
  search->getEntropyCoder()->encodePartSize( data.tempCU, 0, uiDepth, true );
  search->getEntropyCoder()->encodePredInfo( data.tempCU, 0,          true );
  search->getEntropyCoder()->encodeIPCMInfo(data.tempCU, 0, true );

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  search->getEntropyCoder()->encodeCoeff( data.tempCU, 0, uiDepth, data.tempCU->getWidth (0), data.tempCU->getHeight(0), bCodeDQP );
  setdQPFlag( bCodeDQP );
  
  if( m_bUseSBACRD ) search->getRDGoOnSbacCoder()->store(search->getRDSbacCoder()[uiDepth][CI_TEMP_BEST]);
  
  data.tempCU->getTotalBits() = search->getEntropyCoder()->getNumberOfWrittenBits();
  if(m_pcEncCfg->getUseSBACRD())
  {
    data.tempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)search->getEntropyCoder()->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  }
  data.tempCU->getTotalCost() = search->getPcRdCost()->calcRdCost( data.tempCU->getTotalBits(), data.tempCU->getTotalDistortion() );
  
  xCheckDQP( search, data.tempCU );
  xCheckBestMode(search, data, uiDepth);
}

/** Check R-D costs for a CU with PCM mode. 
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 * 
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM( TEncSearch *search, DATA &data )
{
  UInt uiDepth = data.tempCU->getDepth( 0 );

  data.tempCU->setSkipFlagSubParts( false, 0, uiDepth );

  data.tempCU->setIPCMFlag(0, true);
  data.tempCU->setIPCMFlagSubParts (true, 0, data.tempCU->getDepth(0));
  data.tempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
  data.tempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  data.tempCU->setTrIdxSubParts ( 0, 0, uiDepth );
  data.tempCU->setCUTransquantBypassSubParts( m_pcEncCfg->getCUTransquantBypassFlagValue(), 0, uiDepth );

  search->IPCMSearch( data.tempCU, data.origYuv, data.predYuvTemp, data.resiYuvTemp, data.recoYuvTemp);

  if( m_bUseSBACRD ) search->getRDGoOnSbacCoder()->load(search->getRDSbacCoder()[uiDepth][CI_CURR_BEST]);

  search->getEntropyCoder()->resetBits();
  if ( data.tempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    search->getEntropyCoder()->encodeCUTransquantBypassFlag( data.tempCU, 0,          true );
  }
  search->getEntropyCoder()->encodeSkipFlag ( data.tempCU, 0,          true );
  search->getEntropyCoder()->encodePredMode ( data.tempCU, 0,          true );
  search->getEntropyCoder()->encodePartSize ( data.tempCU, 0, uiDepth, true );
  search->getEntropyCoder()->encodeIPCMInfo ( data.tempCU, 0, true );

  if( m_bUseSBACRD ) search->getRDGoOnSbacCoder()->store(search->getRDSbacCoder()[uiDepth][CI_TEMP_BEST]);

  data.tempCU->getTotalBits() = search->getEntropyCoder()->getNumberOfWrittenBits();
  if(m_pcEncCfg->getUseSBACRD())
  {
    data.tempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)search->getEntropyCoder()->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  }
  data.tempCU->getTotalCost() = search->getPcRdCost()->calcRdCost( data.tempCU->getTotalBits(), data.tempCU->getTotalDistortion() );

  xCheckDQP( search, data.tempCU );
  xCheckBestMode( search, data, uiDepth );
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param data.tempCU
 * \returns Void
 */
Void TEncCu::xCheckBestMode( TEncSearch *search, DATA &data, UInt uiDepth )
{
  if( data.tempCU->getTotalCost() < data.bestCU->getTotalCost() )
  {
    TComYuv* pcYuv;
    // Change Information data
    TComDataCU* pcCU = data.bestCU;
    data.bestCU = data.tempCU;
    data.tempCU = pcCU;

    // Change Prediction data
    pcYuv = data.predYuvBest;
    data.predYuvBest = data.predYuvTemp;
    data.predYuvTemp = pcYuv;

    // Change Reconstruction data
    pcYuv = data.recoYuvBest;
    data.recoYuvBest = data.recoYuvTemp;
    data.recoYuvTemp = pcYuv;

    pcYuv = NULL;
    pcCU  = NULL;

    if( m_bUseSBACRD )  // store temp best CI for next CU coding
      search->getRDSbacCoder()[uiDepth][CI_TEMP_BEST]->store(search->getRDSbacCoder()[uiDepth][CI_NEXT_BEST]);
  }
}

Void TEncCu::xCheckDQP( TEncSearch *search, TComDataCU* pcCU )
{
  UInt uiDepth = pcCU->getDepth( 0 );

  if( pcCU->getSlice()->getPPS()->getUseDQP() && (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    if ( pcCU->getCbf( 0, TEXT_LUMA, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_U, 0 ) || pcCU->getCbf( 0, TEXT_CHROMA_V, 0 ) )
    {
#if !RDO_WITHOUT_DQP_BITS
      search->getEntropyCoder()->resetBits();
      search->getEntropyCoder()->encodeQP( pcCU, 0, false );
      pcCU->getTotalBits() += search->getEntropyCoder()->getNumberOfWrittenBits(); // dQP bits
      if(m_pcEncCfg->getUseSBACRD())
      {
        pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)search->getEntropyCoder()->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      }
      pcCU->getTotalCost() = search->getPcRdCost()->calcRdCost( pcCU->getTotalBits(), pcCU->getTotalDistortion() );
#endif
    }
    else
    {
      pcCU->setQPSubParts( pcCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
    }
  }
}

Void TEncCu::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY, DATA &data )
{
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  Bool bSliceEnd   = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx && 
    pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  if(!bSliceEnd && !bSliceStart && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
    UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
    UInt uiBlkWidth    = rpcPic->getNumPartInWidth() >> (uiDepth);
    UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;
    data.recoYuvBest->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
  }
  else
  {
    UInt uiQNumParts = ( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) )>>2;

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      UInt uiSubCULPelX   = uiLPelX + ( g_uiMaxCUWidth >>(uiDepth+1) )*( uiPartUnitIdx &  1 );
      UInt uiSubCUTPelY   = uiTPelY + ( g_uiMaxCUHeight>>(uiDepth+1) )*( uiPartUnitIdx >> 1 );

      Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() && 
        rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xCopyYuv2Pic( rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, pcCU, uiSubCULPelX, uiSubCUTPelY, data );   // Copy Yuv data to picture Yuv
      }
    }
  }
}

Void TEncCu::xCopyYuv2Tmp( DATA &data, DATA &subData, UInt uiPartUnitIdx, UInt uiNextDepth )
{
//  UInt uiCurrDepth = uiNextDepth - 1;
  subData.recoYuvBest->copyToPartYuv( data.recoYuvTemp, uiPartUnitIdx );
}

/** Function for filling the PCM buffer of a CU using its original sample array 
 * \param pcCU pointer to current CU
 * \param pcOrgYuv pointer to original sample array
 * \returns Void
 */
Void TEncCu::xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv )
{

  UInt   width        = pCU->getWidth(0);
  UInt   height       = pCU->getHeight(0);

  Pel*   pSrcY = pOrgYuv->getLumaAddr(0, width); 
  Pel*   pDstY = pCU->getPCMSampleY();
  UInt   srcStride = pOrgYuv->getStride();

  for(Int y = 0; y < height; y++ )
  {
    for(Int x = 0; x < width; x++ )
    {
      pDstY[x] = pSrcY[x];
    }
    pDstY += width;
    pSrcY += srcStride;
  }

  Pel* pSrcCb       = pOrgYuv->getCbAddr();
  Pel* pSrcCr       = pOrgYuv->getCrAddr();;

  Pel* pDstCb       = pCU->getPCMSampleCb();
  Pel* pDstCr       = pCU->getPCMSampleCr();;

  UInt srcStrideC = pOrgYuv->getCStride();
  UInt heightC   = height >> 1;
  UInt widthC    = width  >> 1;

  for(Int y = 0; y < heightC; y++ )
  {
    for(Int x = 0; x < widthC; x++ )
    {
      pDstCb[x] = pSrcCb[x];
      pDstCr[x] = pSrcCr[x];
    }
    pDstCb += widthC;
    pDstCr += widthC;
    pSrcCb += srcStrideC;
    pSrcCr += srcStrideC;
  }
}

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff* rpcCoeff, Int* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples )
{
  for( Int n = 0; n < NumCoeffInCU; n++ )
  {
    Int u = abs( rpcCoeff[ n ] );
    Int absc = rpcArlCoeff[ n ];

    if( u != 0 )
    {
      if( u < LEVEL_RANGE )
      {
        cSum[ u ] += ( Double )absc;
        numSamples[ u ]++;
      }
      else 
      {
        cSum[ LEVEL_RANGE ] += ( Double )absc - ( Double )( u << ARL_C_PRECISION );
        numSamples[ LEVEL_RANGE ]++;
      }
    }
  }

  return 0;
}

/** Collect ARL statistics from one LCU
 * \param pcCU
 */
Void TEncCu::xLcuCollectARLStats(TComDataCU* rpcCU )
{
  Double cSum[ LEVEL_RANGE + 1 ];     //: the sum of DCT coefficients corresponding to datatype and quantization output
  UInt numSamples[ LEVEL_RANGE + 1 ]; //: the number of coefficients corresponding to datatype and quantization output

  TCoeff* pCoeffY = rpcCU->getCoeffY();
  Int* pArlCoeffY = rpcCU->getArlCoeffY();

  UInt uiMinCUWidth = g_uiMaxCUWidth >> g_uiMaxCUDepth;
  UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;

  memset( cSum, 0, sizeof( Double )*(LEVEL_RANGE+1) );
  memset( numSamples, 0, sizeof( UInt )*(LEVEL_RANGE+1) );

  // Collect stats to cSum[][] and numSamples[][]
  for(Int i = 0; i < rpcCU->getTotalNumPart(); i ++ )
  {
    UInt uiTrIdx = rpcCU->getTransformIdx(i);

    if(rpcCU->getPredictionMode(i) == MODE_INTER)
    if( rpcCU->getCbf( i, TEXT_LUMA, uiTrIdx ) )
    {
      xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
    }//Note that only InterY is processed. QP rounding is based on InterY data only.
   
    pCoeffY  += uiMinNumCoeffInCU;
    pArlCoeffY  += uiMinNumCoeffInCU;
  }

  for(Int u=1; u<LEVEL_RANGE;u++)
  {
    m_pcTrQuant->getSliceSumC()[u] += cSum[ u ] ;
    m_pcTrQuant->getSliceNSamples()[u] += numSamples[ u ] ;
  }
  m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[ LEVEL_RANGE ] ;
  m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[ LEVEL_RANGE ] ;
}
#endif
//! \}
