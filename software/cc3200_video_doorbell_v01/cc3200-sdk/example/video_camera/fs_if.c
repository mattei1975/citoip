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

#include "simplelink.h"
#include "fs_if.h"

/*!
 */
cc_s32 sl_OpenFile(cc_u8 *fName, cc_s32 *fHdl)
{
    cc_u32  token = 0;
    cc_s32  retVal = -1;

    retVal = sl_FsOpen(fName, FS_MODE_OPEN_READ, &token, fHdl);
    return retVal;
}

/*!
 */
cc_s32 sl_GetFileSize(cc_u8 *fName, cc_u32 *fSize)
{
    SlFsFileInfo_t fInfo = {0};

    cc_u32 token    = 0;
    cc_s32 retVal   = -1;

    retVal = sl_FsGetInfo(fName, token, &fInfo);
    if(retVal >= 0)
    {
        *fSize = fInfo.FileLen;
    }

    return retVal;
}

/*!
 */
cc_s32 sl_GetData(cc_s32 fHdl, cc_u8 *pBuff,
                cc_u32 length, cc_u32 offset)
{
    cc_s32 retVal = -1;

    retVal = sl_FsRead(fHdl, offset, pBuff, length);
    return  retVal;
}

/*!
 */
cc_s32 sl_CloseFile(cc_s32 fHdl)
{
    cc_s32 retVal = -1;

    retVal = sl_FsClose(fHdl, 0, 0, 0);
    return retVal;
}
