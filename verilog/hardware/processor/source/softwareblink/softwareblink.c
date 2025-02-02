volatile unsigned int *gDebugLedsMemoryMappedRegister = (unsigned int *)0x2000;
enum {
	kSpinDelay = 400000,
};

int main(void)
{


	/*
	 *	Reading from the special address 0x2000 will cause sail to
	 *	set the value of 8 of the FPGA's pins to the byte written
	 *	to the address. See the PCF file for how those 8 pins are
	 *	mapped.
	 */
	/*
	 *	Reading from the special address pointed to by
	 *	kDebugLedsMemoryMappedRegister will cause the processor to
	 *	set the value of 8 of the FPGA's pins to the byte written
	 *	to the address. See the PCF file for how those 8 pins are
	 *	mapped.
	 */
	while (1)
	{

		/*
		 *	Spin
		 */
		for (int j = 0; j < kSpinDelay; j++)
		{
		}

		*gDebugLedsMemoryMappedRegister = 0x00;

		for (int j = 0; j < kSpinDelay; j++)
		{
		}

		*gDebugLedsMemoryMappedRegister = 0xFF;
	}
}
