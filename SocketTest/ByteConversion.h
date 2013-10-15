#pragma once



class ByteConversion
{
	enum DataStruct{Hand, Finger, Head, Touch};

public: 
	struct TestStruct
	{
	public:
		int a;
		float b;
	};


public:
	ByteConversion(void);
	~ByteConversion(void);
	/*char* ByteConversion::intToBytes(int paramInt);*/
	void ByteConversion::intToBytes(int paramInt,char* byteArray);

	char* HandConversion(TestStruct testStruct);


};

