#include "ardrone_sdk.h"

C_RESULT navdata_init( void* private_data )
{
  //Your code to initialize your local variables.
  return C_OK;
}

C_RESULT navdata_process( const navdata_unpacked_t* const pnd )
{
  //Retrieves current Navdata unpacked.
  printf("%d\n",pnd->navdata_demo.vbat_flying_percentage);
  return C_OK;
}

C_RESULT navdata_release( void )
{
  //Free local variables.
  return C_OK;
}
