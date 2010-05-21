/*	! \file src/edp/ati6284/osiUserCode.cc
    * \brief NI osiUserCode.cpp holds the two user defined functions
    *  needed to port iBus to a target platform.
    *
    *  acquireBoard( ) -- constructs and initializes the iBus
    *
    *  releaseBoard( ) -- deletes and cleans up the iBus
    *
    *  There are also assorted helper functions which are also
    *  platform specific.
    */

//!< Platform independent headers
#include <stdint.h>
#include <string.h>

#include <hw/pci.h>
#include <sys/mman.h>
#include <sys/neutrino.h>

#include "osiBus.h"
#include "display.h"

static int pidx;
static void* hdl;			//!< wlasciwy uchwyt do danego urzadzenia
static int phdl;			//!< pci handle -> fd do servera PCI
static void *mem0,*mem1;
static struct pci_dev_info info;
static int id;				//! Board interrupt id

namespace mrrocpp {
namespace edp {
namespace sensor {

extern const struct sigevent *isr_handler (void *bus, int id);
extern struct sigevent ati6284event;

} // namespace sensor
} // namespace edp
} // namespace mrrocpp

/*  ! \fn acquireBoard
    * \brief Return iBus of first matching PCI/PXI card.
*/
iBus* acquireBoard(const u32 devicePCI_ID)
{
	u32 devBAR0,BAR0sz,devBAR1,BAR1sz;
	iBus *bus;
	
	//!< Find the PCI BAR memory ranges
	
	memset( &info, 0, sizeof( info ) );//!<  obsluga przerwania
	
	//!<  Connect to the PCI server
	phdl = pci_attach( 0 );
	if( phdl == -1 ) {
		fprintf( stderr, "Unable to initialize PCI\n" );
		exit( EXIT_FAILURE ) ;
	}
#if DEBUG
	printf("Program testowy PCI-6034E\n");
#endif
	
	//!< Memory map the BARs to get access to the PCI card's memory
	//!< Initialize the pci_dev_info structure

	pidx = 0;
	info.VendorId = 0x1093; // National Instruments
	info.DeviceId = 0x2CA0; // PCI-6034E

	//!< dolaczamy driver do urzadzenia na PCI
	hdl = pci_attach_device( NULL, PCI_INIT_ALL, pidx, &info );

	if( hdl == NULL ) {
		fprintf( stderr, "Unable to locate NI-6034E\n" );
		exit( EXIT_FAILURE ) ;
	}
	else {
#if DEBUG
		printf("connected to NI-6034E\n");
		for (int i = 0; i < 6; i++) {
					if (info.BaseAddressSize[i] > 0)
					printf("Aperture %d: Base 0x%llx Length %d bytes Type %s\n", i,
					PCI_IS_MEM(info.CpuBaseAddress[i]) ?PCI_MEM_ADDR(info.CpuBaseAddress[i]) : PCI_IO_ADDR(info.CpuBaseAddress[i]),
					info.BaseAddressSize[i], PCI_IS_MEM(info.CpuBaseAddress[i]) ? "MEM" : "IO");
		}
#endif

		devBAR0=(u32)info.CpuBaseAddress[0];	
		devBAR1=(u32)info.CpuBaseAddress[1];

		mem0 = mmap_device_memory( NULL, info.BaseAddressSize[0], PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, info.CpuBaseAddress[0] );
		mem1 = mmap_device_memory( NULL, info.BaseAddressSize[1], PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, info.CpuBaseAddress[1] );

		if ( mem0 == MAP_FAILED ) {
				perror( "mmap_device_memory for physical address 0 failed" );
				exit( EXIT_FAILURE );
		}
		if ( mem1 == MAP_FAILED ) {
				perror( "mmap_device_memory for physical address 1 failed" );
				exit( EXIT_FAILURE );
		}
#if DEBUG
		printf("Memory mapped address 0 <0x%llx> of size <%d> CPU <%d> to address of <0x%llx>.\n",info.PciBaseAddress[0],info.BaseAddressSize[0],info.BaseAddressSize[0], mem0);
		printf("Memory mapped address 1 <0x%llx> of size <%d> CPU <%d> to address of <0x%llx>.\n",info.PciBaseAddress[1],info.BaseAddressSize[1],info.BaseAddressSize[0], mem1);
#endif
	}


	//!< create a new iBus which uses the memory mapped addresses
	bus = new iBus(0, 0, mem0, mem1);

	bus->_physBar[0] = (u32)devBAR0;
	bus->_physBar[1] = (u32)devBAR1;
	bus->_physBar[2] = (u32)NULL;
	bus->_physBar[3] = (u32)NULL;
	bus->_physBar[4] = (u32)NULL;
	bus->_physBar[5] = (u32)NULL;
#if DEBUG	
	printf("InterruptAttach\n");
#endif	
#if INTERRUPT
	memset( &mrrocpp::edp::sensor::ati6284event, 0, sizeof( mrrocpp::edp::sensor::ati6284event ) );///!< obsluga przerwania
	SIGEV_INTR_INIT( &mrrocpp::edp::sensor::ati6284event );
	if (( id = InterruptAttach (info.Irq, mrrocpp::edp::sensor::isr_handler, NULL , NULL , 0)) == -1)
		printf("Error InterruptAttach\n");
#if DEBUG
	else
		printf("InterruptAttach OK\n");
#endif	
#endif

	return bus;
}

void releaseBoard(iBus *&bus)
{
#if INTERRUPT
	InterruptDetach (id);
#endif

	//!< unmap the memory and close whatever system resources

	if(munmap_device_memory( mem0, info.BaseAddressSize[0])==-1 )
		perror( "munmap_device_memory for physical address 0 failed" );

	if(munmap_device_memory( mem1, info.BaseAddressSize[1] )==-1)
		perror( "munmap_device_memory for physical address 1 failed" );

	//!< were used to create the iBus
	pci_detach_device( hdl ); //!< odlacza driver od danego urzadzenia na PCI
	//!< Disconnect from the PCI server
	pci_detach( phdl );
#if DEBUG
	printf("Odlaczono uklad PCI-6034E\n");
#endif
	delete bus;
}
