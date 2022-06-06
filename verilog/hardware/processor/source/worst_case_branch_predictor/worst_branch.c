int
main(void)
{
	enum
	{
		kSpinDelay = 400000,
	};

	volatile unsigned int *gDebugLedsMemoryMappedRegister = (unsigned int *)0x2000;
	/*
	 *	Reading from the special address pointed to by
	 *	kDebugLedsMemoryMappedRegister will cause the processor to
	 *	set the value of 8 of the FPGA's pins to the byte written
	 *	to the address. See the PCF file for how those 8 pins are
	 *	mapped.
	 */
	int p=0;
	*gDebugLedsMemoryMappedRegister = ~(*gDebugLedsMemoryMappedRegister);

	for (int j = 0; j < kSpinDelay; j++) {
		for (int j2 = 0; j2 < p; j2++);
	};
}
