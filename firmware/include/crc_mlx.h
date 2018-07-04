/**
 * \file
 * Functions and types for CRC checks.
 *
 * Generated on Thu Jun 21 19:28:07 2018
 * by pycrc v0.9.1, https://pycrc.org
 * using the configuration:
 *  - Width         = 8
 *  - Poly          = 0x2f
 *  - XorIn         = 0xff
 *  - ReflectIn     = False
 *  - XorOut        = 0xff
 *  - ReflectOut    = False
 *  - Algorithm     = table-driven
 *
 * This file defines the functions crc_mlx_init(), crc_mlx_update() and crc_mlx_finalize().
 *
 * The crc_mlx_init() function returns the inital \c crc value and must be called
 * before the first call to crc_mlx_update().
 * Similarly, the crc_mlx_finalize() function must be called after the last call
 * to crc_mlx_update(), before the \c crc is being used.
 * is being used.
 *
 * The crc_mlx_update() function can be called any number of times (including zero
 * times) in between the crc_mlx_init() and crc_mlx_finalize() calls.
 *
 * This pseudo-code shows an example usage of the API:
 * \code{.c}
 * crc_mlx_t crc;
 * unsigned char data[MAX_DATA_LEN];
 * size_t data_len;
 *
 * crc = crc_mlx_init();
 * while ((data_len = read_data(data, MAX_DATA_LEN)) > 0) {
 *     crc = crc_mlx_update(crc, data, data_len);
 * }
 * crc = crc_mlx_finalize(crc);
 * \endcode
 */
#ifndef _CRC_MLX_H_
#define _CRC_MLX_H_

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * The definition of the used algorithm.
 *
 * This is not used anywhere in the generated code, but it may be used by the
 * application code to call algorithm-specific code, if desired.
 */
#define CRC_ALGO_TABLE_DRIVEN 1


/**
 * The type of the CRC values.
 *
 * This type must be big enough to contain at least 8 bits.
 */
typedef uint_fast8_t crc_mlx_t;


/**
 * Calculate the initial crc value.
 *
 * \return     The initial crc value.
 */
static inline crc_mlx_t crc_mlx_init(void)
{
    return 0xff;
}


/**
 * Update the crc value with new data.
 *
 * \param[in] crc      The current crc value.
 * \param[in] data     Pointer to a buffer of \a data_len bytes.
 * \param[in] data_len Number of bytes in the \a data buffer.
 * \return             The updated crc value.
 */
crc_mlx_t crc_mlx_update(crc_mlx_t crc, const void *data, size_t data_len);


/**
 * Calculate the final crc value.
 *
 * \param[in] crc  The current crc value.
 * \return     The final crc value.
 */
static inline crc_mlx_t crc_mlx_finalize(crc_mlx_t crc)
{
    return crc ^ 0xff;
}


#ifdef __cplusplus
}           /* closing brace for extern "C" */
#endif

#endif      /* _CRC_MLX_H_ */
