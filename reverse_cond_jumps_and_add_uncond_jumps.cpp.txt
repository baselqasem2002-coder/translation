
/*! @file
 *  This is an example of the PIN tool that demonstrates some basic PIN APIs 
 *  and could serve as the starting point for developing your first PIN tool
 */


/*
Flags:
------
CF - carry flag    - Set on high-order bit carry or borrow; cleared otherwise
PF - parity flag   - Set if low-order eight bits of result contain an even number of "1" bits; cleared otherwise
ZF - zero flags    - Set if result is zero; cleared otherwise
SF - sign flag     -  Set equal to high-order bit of result (0 if positive 1 if negative)
OF - overflow flag -  Set if result is too large a positive number or too small a negative number (excluding sign bit) to fit in destination operand; cleared otherwise

Table of x86 condtional jumps:
---------------------------------------------------
Opcode: Instr::     Description:
77 cb	JA rel8		Jump short if above (CF=0 and ZF=0).				(same as JNBE)
73 cb	JAE rel8	Jump short if above or equal (CF=0).				(same as JNB or JNC)
72 cb	JB rel8		Jump short if below (CF=1).							(same as JNAE or JC)
76 cb	JBE rel8	Jump short if below or equal (CF=1 or ZF=1).		(same as JNA)
72 cb	JC rel8		Jump short if carry (CF=1).							(same as JB or JNAE)
E3 cb	JCXZ rel8	Jump short if CX register is 0.
E3 cb	JECXZ rel8	Jump short if ECX register is 0.
E3 cb	JRCXZ rel8	Jump short if RCX register is 0.
74 cb	JE rel8		Jump short if equal (ZF=1).							(same as JZ)
7F cb	JG rel8		Jump short if greater (ZF=0 and SF=OF).				(same as JNLE)
7D cb	JGE rel8	Jump short if greater or equal (SF=OF).				(same as JNL)
7C cb	JL rel8		Jump short if less (SF != OF).						(same as JNGE)
7E cb	JLE rel8	Jump short if less or equal (ZF=1 or SF != OF).		(same as JNG)
76 cb	JNA rel8	Jump short if not above (CF=1 or ZF=1).				(same as JBE)
72 cb	JNAE rel8	Jump short if not above or equal (CF=1).			(same as JB or JC)
73 cb	JNB rel8	Jump short if not below (CF=0).						(same as JAE or JNC)
77 cb	JNBE rel8	Jump short if not below or equal (CF=0 and ZF=0).	(same as JA)
73 cb	JNC rel8	Jump short if not carry (CF=0).						(same as JAE or JNB)
75 cb	JNE rel8	Jump short if not equal (ZF=0).						(same as JNZ)
7E cb	JNG rel8	Jump short if not greater (ZF=1 or SF != OF).		(same as JLE)
7C cb	JNGE rel8	Jump short if not greater or equal (SF !=  OF).		(same as JL)
7D cb	JNL rel8	Jump short if not less (SF=OF).						(same as JGE)
7F cb	JNLE rel8	Jump short if not less or equal (ZF=0 and SF=OF).	(same as JG)
71 cb	JNO rel8	Jump short if not overflow (OF=0).
7B cb	JNP rel8	Jump short if not parity (PF=0).					(same as JPO)
79 cb	JNS rel8	Jump short if not sign (SF=0).
75 cb	JNZ rel8	Jump short if not zero (ZF=0).						(same as JNE)
70 cb	JO rel8		Jump short if overflow (OF=1).
7A cb	JP rel8		Jump short if parity (PF=1).						(same as JPE)
7A cb	JPE rel8	Jump short if parity even (PF=1).					(same as JP)
7B cb	JPO rel8	Jump short if parity odd (PF=0).					(same as JNP)
78 cb	JS rel8		Jump short if sign (SF=1).
74 cb	JZ rel8		Jump short if zero (ZF=1).							(same as JE)


XED ICLASSes for COnd Jumos:
--------------------------
  XED_ICLASS_JB, 
  XED_ICLASS_JBE, 
  XED_ICLASS_JL, 
  XED_ICLASS_JLE, 
  XED_ICLASS_JNB, 
  XED_ICLASS_JNBE, 
  XED_ICLASS_JNL, 
  XED_ICLASS_JNLE, 
  XED_ICLASS_JNO, 
  XED_ICLASS_JNP, 
  XED_ICLASS_JNS, 
  XED_ICLASS_JNZ, 
  XED_ICLASS_JO, 
  XED_ICLASS_JP, 
  XED_ICLASS_JRCXZ, 
  XED_ICLASS_JS, 
  XED_ICLASS_JZ, 

*/



#include "pin.H"
#include <iostream>
#include <fstream>

extern "C" {
#include "xed-interface.h"
}

using namespace std;

/* ================================================================== */
// Global variables 
/* ================================================================== */

std::ostream * out = &cerr;

// For XED:
#if defined(TARGET_IA32E)
    xed_state_t dstate = {XED_MACHINE_MODE_LONG_64, XED_ADDRESS_WIDTH_64b};
#else
    xed_state_t dstate = { XED_MACHINE_MODE_LEGACY_32, XED_ADDRESS_WIDTH_32b};
#endif

/* ===================================================================== */
// Command line switches
/* ===================================================================== */
KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE,  "pintool",
    "o", "", "specify file name for MyPinTool output");


/* ===================================================================== */
// Utilities
/* ===================================================================== */

/*!
 *  Print out help message.
 */
INT32 Usage()
{
    cerr << "This tool detect info on conditional jumps" << endl;

    cerr << KNOB_BASE::StringKnobSummary() << endl;

    return -1;
}

/* ===================================================================== */
// Analysis routines
/* ===================================================================== */



VOID Trace(TRACE trace, VOID *v)
{
    // Visit every basic block in the trace
    for (BBL bbl = TRACE_BblHead(trace); BBL_Valid(bbl); bbl = BBL_Next(bbl))
    {
        INS ins_tail = BBL_InsTail(bbl);
	
        xed_decoded_inst_t *xedd = INS_XedDec(ins_tail);
        
		xed_category_enum_t category_enum = xed_decoded_inst_get_category(xedd);

		if (category_enum != XED_CATEGORY_COND_BR) 
			continue;

		xed_iclass_enum_t iclass_enum = xed_decoded_inst_get_iclass(xedd);

  		if (iclass_enum == XED_ICLASS_JRCXZ)
			continue;    // do not revert JRCXZ

		xed_iclass_enum_t 	retverted_iclass;

		switch (iclass_enum) {

			case XED_ICLASS_JB:
				retverted_iclass = XED_ICLASS_JNB;		
				break;

			case XED_ICLASS_JBE:
				retverted_iclass = XED_ICLASS_JNBE;
				break;

			case XED_ICLASS_JL:
				retverted_iclass = XED_ICLASS_JNL;
				break;
		
			case XED_ICLASS_JLE:
				retverted_iclass = XED_ICLASS_JNLE;
				break;

			case XED_ICLASS_JNB: 
			    retverted_iclass = XED_ICLASS_JB;
				break;

			case XED_ICLASS_JNBE: 
				retverted_iclass = XED_ICLASS_JBE;
				break;

			case XED_ICLASS_JNL:
			retverted_iclass = XED_ICLASS_JL;
				break;

			case XED_ICLASS_JNLE:
				retverted_iclass = XED_ICLASS_JLE;
				break;

			case XED_ICLASS_JNO:
				retverted_iclass = XED_ICLASS_JO;
				break;

			case XED_ICLASS_JNP: 
				retverted_iclass = XED_ICLASS_JP;
				break;

			case XED_ICLASS_JNS: 
				retverted_iclass = XED_ICLASS_JS;
				break;

			case XED_ICLASS_JNZ:
				retverted_iclass = XED_ICLASS_JZ;
				break;

			case XED_ICLASS_JO:
				retverted_iclass = XED_ICLASS_JNO;
				break;

			case XED_ICLASS_JP: 
			    retverted_iclass = XED_ICLASS_JNP;
				break;

			case XED_ICLASS_JS: 
				retverted_iclass = XED_ICLASS_JNS;
				break;

			case XED_ICLASS_JZ:
				retverted_iclass = XED_ICLASS_JNZ;
				break;
	
			default:
				continue;
		}

		// Converts the decoder request to a valid encoder request:
		xed_encoder_request_init_from_decode (xedd);

		// set the reverted opcode;
		xed_encoder_request_set_iclass	(xedd, retverted_iclass);

		xed_uint8_t enc_buf[XED_MAX_INSTRUCTION_BYTES];
		unsigned int max_size = XED_MAX_INSTRUCTION_BYTES;
		unsigned int new_size = 0;
    
		xed_error_enum_t xed_error = xed_encode (xedd, enc_buf, max_size, &new_size);
		if (xed_error != XED_ERROR_NONE) {
			cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) <<  endl;
			continue;
		}		

		
		// create a direct uncond jump to the same address:
		xed_uint8_t enc_buf2[XED_MAX_INSTRUCTION_BYTES];		
		xed_int32_t disp = xed_decoded_inst_get_branch_displacement(xedd);
		xed_encoder_instruction_t  enc_instr;

		xed_inst1(&enc_instr, dstate, 
				XED_ICLASS_JMP, 64,
				xed_relbr(disp, 32));
                                
		xed_encoder_request_t enc_req;

		xed_encoder_request_zero_set_mode(&enc_req, &dstate);
		xed_bool_t convert_ok = xed_convert_to_encoder_request(&enc_req, &enc_instr);
		if (!convert_ok) {
			cerr << "conversion to encode request failed" << endl;
			continue;
		}

		xed_error = xed_encode (&enc_req, enc_buf2, max_size, &new_size);
		if (xed_error != XED_ERROR_NONE) {
			cerr << "ENCODE ERROR: " << xed_error_enum_t2str(xed_error) << endl;				
			continue;
		}		
        

		//print the original and the new reverted cond instructions:
		//
		cerr << "orig instr:              " << hex << INS_Address(ins_tail) << " " << INS_Disassemble(ins_tail) << endl;		         

		char buf[2048];		
		xed_decoded_inst_t new_xedd;
		xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);

		xed_error_enum_t xed_code = xed_decode(&new_xedd, enc_buf, XED_MAX_INSTRUCTION_BYTES);
		if (xed_code != XED_ERROR_NONE) {
			cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << INS_Address(ins_tail) << endl;
			continue;
		}

		xed_format_context(XED_SYNTAX_INTEL, &new_xedd, buf, 2048, INS_Address(ins_tail), 0, 0);
		cerr << "reverted cond jump:      " << hex << INS_Address(ins_tail) << " " << buf << endl;		         

		xed_decoded_inst_zero_set_mode(&new_xedd,&dstate);
		xed_code = xed_decode(&new_xedd, enc_buf2, XED_MAX_INSTRUCTION_BYTES);
		if (xed_code != XED_ERROR_NONE) {
			cerr << "ERROR: xed decode failed for instr at: " << "0x" << hex << INS_Address(ins_tail) << endl;
			continue;
		}

		xed_format_context(XED_SYNTAX_INTEL, &new_xedd, buf, 2048, INS_Address(ins_tail), 0, 0);
		cerr << "newly added uncond jump: " << hex << INS_Address(ins_tail) << " " << buf << endl << endl;
    }



}

/*!
 * Print out analysis results.
 * This function is called when the application exits.
 * @param[in]   code            exit code of the application
 * @param[in]   v               value specified by the tool in the 
 *                              PIN_AddFiniFunction function call
 */
VOID Fini(INT32 code, VOID *v)
{
    *out <<  "===============================================" << endl;
}

/*!
 * The main procedure of the tool.
 * This function is called when the application image is loaded but not yet started.
 * @param[in]   argc            total number of elements in the argv array
 * @param[in]   argv            array of command line arguments, 
 *                              including pin -t <toolname> -- ...
 */
int main(int argc, char *argv[])
{
    // Initialize PIN library. Print help message if -h(elp) is specified
    // in the command line or the command line is invalid 
    if( PIN_Init(argc,argv) )
    {
        return Usage();
    }
    
    string fileName = KnobOutputFile.Value();

    if (!fileName.empty()) { out = new std::ofstream(fileName.c_str());}

    // Register function to be called to instrument traces
    TRACE_AddInstrumentFunction(Trace, 0);

    if (!KnobOutputFile.Value().empty()) 
    {
        cerr << "See file " << KnobOutputFile.Value() << " for analysis results" << endl;
    }
    cerr <<  "===============================================" << endl;

    // Start the program, never returns
    PIN_StartProgram();
    
    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
