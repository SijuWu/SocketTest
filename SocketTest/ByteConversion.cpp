#include "StdAfx.h"
#include "ByteConversion.h"




ByteConversion::ByteConversion(void)
{
}


ByteConversion::~ByteConversion(void)
{
}

//Function used to convert Int to Byte
void ByteConversion::intToBytes(int paramInt,char* byteArray)
{
	for(int i=0;i<4;++i)
		byteArray[i]=(paramInt>>(i*8));
}

char* ByteConversion::HandConversion(TestStruct testStruct)
{
	char structByte[8];
	char* aByte=reinterpret_cast<char *>(&testStruct.a);
	char* bByte=reinterpret_cast<char *>(&testStruct.b);

	memmove(structByte,aByte,4);
	memmove(&structByte[4],bByte,4);

	return structByte;
}