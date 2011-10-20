/*
 * gateway.cc
 *
 *  Created on: May 14, 2011
 *      Author: ptroja
 */

#include "gateway.h"

namespace mrrocpp {
namespace edp {
namespace canopen {

/*
 *************************************************************
 check EPOS error code
 ****************************************************************
 */

/* check the global variable E_error for EPOS error code */
void gateway::checkEPOSerror(DWORD E_error)
{
	const char *msg;
	switch (E_error) {
		//case 0x00000000: msg = "No Communication Error: RS232 communication successful"; break;
		case 0x00000000: return;
		case 0x05030000: msg = "Toggle Error: Toggle bit not alternated"; break;
		case 0x05040000: msg = "SDO Time Out: SDO protocol timed out"; break;
		case 0x05040001: msg = "Client / Server Specifier Error: Client / server command specifier not valid or unknown"; break;
		case 0x05040005: msg = "Out of Memory Error: Out of memory"; break;
		case 0x06010000: msg = "Access Error: Unsupported access to an object"; break;
		case 0x06010001: msg = "Write Only: Read command to a write only object"; break;
		case 0x06010002: msg = "Read Only: Write command to a read only object"; break;
		case 0x06020000: msg = "Object does not exist Error: Last read or write command had wrong object index or subindex"; break;
		case 0x06040041: msg = "PDO mapping Error: Object is not mappable to the PDO"; break;
		case 0x06040042: msg = "PDO Length Error: Number and length of objects to be mapped would exceed PDO length"; break;
		case 0x06040043: msg = "General Parameter Error: General parameter incompatibility"; break;
		case 0x06040047: msg = "General internal Incompatibility Error: General internal incompatibility in device"; break;
		case 0x06060000: msg = "Hardware Error: Access failed due to hardware error"; break;
		case 0x06070010: msg = "Service Parameter Error: Data type does not match, length or service parameter does not match"; break;
		case 0x06070012: msg = "Service Parameter too long Error: Data type does not match, length of service parameter too high"; break;
		case 0x06070013: msg = "Service Parameter too short Error: Data type does not match, length of service parameter too low"; break;
		case 0x06090011: msg = "Object Subindex Error: Last read or write command had wrong object subindex"; break;
		case 0x06090030: msg = "Value Range Error: Value range of parameter exceeded"; break;
		case 0x06090031: msg = "Value too high Error: Value of parameter written too high"; break;
		case 0x06090032: msg = "Value too low Error: Value of parameter written too low"; break;
		case 0x06090036: msg = "Maximum less Minimum Error: Maximum value is less than minimum value"; break;
		case 0x08000000: msg = "General Error: General error"; break;
		case 0x08000020: msg = "Transfer or store Error: Data cannot be transferred or stored"; break;
		case 0x08000021: msg = "Local Control Error: Data cannot be transferred or stored to application because of local control"; break;
		case 0x08000022: msg = "Wrong Device State: Data cannot be transferred or stored to application because of present device state"; break;
		case 0x0F00FFC0: msg = "Wrong NMT State Error: Device is in wrong NMT state"; break;
		case 0x0F00FFBF: msg = "Illegal Command Error: RS232 command is illegal (does not exist)"; break;
		case 0x0F00FFBE: msg = "Password Error: Password is incorrect"; break;
		case 0x0F00FFBC: msg = "Error Service Mode: Device is not in service mode"; break;
		case 0x0F00FFB9: msg = "Error CAN ID: Wrong CAN ID"; break;

		default:
			msg = "unknown EPOS error code"; //TODO: %x\n", E_error);
			printf("EPOS error code: 0x%08x\n", E_error);
			break;
	}

	BOOST_THROW_EXCEPTION(se_canopen_error() << reason(msg));
}

/* copied from EPOS Communication Guide, p.8 */
WORD gateway::CalcFieldCRC(const WORD *pDataArray, WORD numberOfWords)
{
	WORD shifter, c;
	WORD carry;
	WORD CRC = 0;

	if (debug) {
		int i = numberOfWords;
		const WORD * ptr = pDataArray;
		printf("CRC[%d]: ", numberOfWords);
		while(i--) {
			printf("0x%04X ", *ptr++);
		}
		printf("\n");
	}

	//Calculate pDataArray Word by Word
	while (numberOfWords--) {
		shifter = 0x8000; //Initialize BitX to Bit15
		c = *pDataArray++; //Copy next DataWord to c
		do {
			carry = CRC & 0x8000; //Check if Bit15 of CRC is set
			CRC <<= 1; //CRC = CRC * 2
			if (c & shifter)
				CRC++; //CRC = CRC + 1, if BitX is set in c
			if (carry)
				CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
			shifter >>= 1; //Set BitX to next lower Bit,
			//shifter = shifter/2
		} while (shifter);
	}

	if (debug) {
		printf("checksum == %#06x\n", CRC);
	}
	return CRC;
}

} /* namespace epos */
} /* namespace edp */
} /* namespace mrrocpp */
