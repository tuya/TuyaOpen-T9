#ifndef __TKL_AUD_PM_H__
#define __TKL_AUD_PM_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef TKL_AUD_PM_EXT_IMPL
VOID_T tkl_aud_pm_acquire(VOID_T);
VOID_T tkl_aud_pm_release(VOID_T);
#else
static inline VOID_T tkl_aud_pm_acquire(VOID_T)
{
}

static inline VOID_T tkl_aud_pm_release(VOID_T)
{
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* __TKL_AUD_PM_H__ */
