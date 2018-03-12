/*
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __FS_IF_H__
#define __FS_IF_H__

#ifdef    __cplusplus
extern "C" {
#endif

#include "cc_pal_app.h"

/*!
    \brief Open the file containing the firmware

    \param[in]  fName   :   Pointer to file name to be opened
    \param[out] fHdl    :   File handler for the opened file

    \return     0 on success, negative error-code on error

    \note
*/
cc_s32 sl_OpenFile(cc_u8 *fName, cc_s32 *fHdl);

/*!
    \brief Get the size of the file

    \param[in]  fName   :   Pointer to file name to be opened
    \param[out] fHdl    :   Size of the file

    \return     0 on success, negative error-code on error

    \note
*/
cc_s32 sl_GetFileSize(cc_u8 *fName, cc_u32 *fSize);

/*!
    \brief Get the data from the file

    \param[in]  fHdl    :   File handler
    \param[out] fHdl    :   Pointer to the buffer for reading data
    \param[in]  length  :   Size of the data to be read
    \param[in]  offset  :   Offset location for the data to be read

    \return     0 on success, negative error-code on error

    \note
*/
cc_s32 sl_GetData(cc_s32 fHdl, cc_u8 *pBuff, cc_u32 length,  cc_u32 offset);

/*!
    \brief Close the opened file

    \param[in]  fHdl   :   File handler for the opened file

    \return     0 on success, negative error-code on error

    \note
*/
cc_s32 sl_CloseFile(cc_s32 fHdl);


#ifdef  __cplusplus
}
#endif /* __cplusplus */
#endif /*  __FS_IF_H__ */
