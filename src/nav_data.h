#ifndef _NAV_DATA_H_
#define _NAV_DATA_H_

C_RESULT navdata_init( void* private_data );
C_RESULT navdata_process( const navdata_unpacked_t* const pnd );
C_RESULT navdata_release( void );

#endif

