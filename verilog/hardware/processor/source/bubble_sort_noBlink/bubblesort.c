int main(void)
{
	unsigned char bsort_input[] =
		{
			0x53,
			0x69,
			0x6e,
			0x67,
			0x20,
			0x74,
			0x6f,
			0x20,
			0x6d,
			0x65,
			0x20,
			0x6f,
			0x66,
			0x20,
			0x74,
			0x68,
			0x65,
			0x20,
			0x6d,
			0x61,
			0x6e,
			0x2c,
			0x20,
			0x4d,
			0x75,
			0x73,
			0x65,
			0x2c,
			0x20,
			0x74,
			0x68,
			0x65,
			0x20,
			0x6d,
			0x61,
			0x6e,
			0x20,
			0x6f,
			0x66,
			0x20,
			0x74,
			0x77,
			0x69,
			0x73,
			0x74,
			0x73,
			0x20,
			0x61,
			0x6e,
			0x64,
			0x20,
			0x74,
			0x75,
			0x72,
			0x6e,
			0x73,
			0x2e,
			0x2e,
			0x2e,
		};

	const int bsort_input_len = 0x3b;
	int i;
	int maxindex = bsort_input_len - 1;

	while (maxindex > 0)
	{
		for (i = 0; i < maxindex; i++)
		{
			if (bsort_input[i] > bsort_input[i + 1])
			{
				/*		swap		*/
				bsort_input[i] ^= bsort_input[i + 1];
				bsort_input[i + 1] ^= bsort_input[i];
				bsort_input[i] ^= bsort_input[i + 1];
			}
		}

		maxindex--;
	}

	return 0;
}
