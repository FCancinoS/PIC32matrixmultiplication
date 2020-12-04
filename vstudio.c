#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <windows.h>
#include <SETUPAPI.H>

//----------------------------------------------
#define RICH_VENDOR_ID			0x0000
#define RICH_USBHID_GENIO_ID	0x2019

#define INPUT_REPORT_SIZE	64
#define OUTPUT_REPORT_SIZE	64

float matrix1[10][10];
float matrix2[10][10];
float matrix3[10][10];

typedef struct _HIDD_ATTRIBUTES {
	ULONG   Size; // = sizeof (struct _HIDD_ATTRIBUTES)
	USHORT  VendorID;
	USHORT  ProductID;
	USHORT  VersionNumber;
} HIDD_ATTRIBUTES, *PHIDD_ATTRIBUTES;

typedef VOID(__stdcall *PHidD_GetProductString)(HANDLE, PVOID, ULONG);
typedef VOID(__stdcall *PHidD_GetHidGuid)(LPGUID);
typedef BOOLEAN(__stdcall *PHidD_GetAttributes)(HANDLE, PHIDD_ATTRIBUTES);
typedef BOOLEAN(__stdcall *PHidD_SetFeature)(HANDLE, PVOID, ULONG);
typedef BOOLEAN(__stdcall *PHidD_GetFeature)(HANDLE, PVOID, ULONG);

//----------------------------------------------

HINSTANCE                       hHID = NULL;
PHidD_GetProductString          HidD_GetProductString = NULL;
PHidD_GetHidGuid                HidD_GetHidGuid = NULL;
PHidD_GetAttributes             HidD_GetAttributes = NULL;
PHidD_SetFeature                HidD_SetFeature = NULL;
PHidD_GetFeature                HidD_GetFeature = NULL;
HANDLE                          DeviceHandle = INVALID_HANDLE_VALUE;

unsigned int moreHIDDevices = TRUE;
unsigned int HIDDeviceFound = FALSE;

unsigned int terminaAbruptaEInstantaneamenteElPrograma = 0;



void Load_HID_Library(void) {
	hHID = LoadLibrary("HID.DLL");
	if (!hHID) {
		printf("Failed to load HID.DLL\n");
		return;
	}

	HidD_GetProductString = (PHidD_GetProductString)GetProcAddress(hHID, "HidD_GetProductString");
	HidD_GetHidGuid = (PHidD_GetHidGuid)GetProcAddress(hHID, "HidD_GetHidGuid");
	HidD_GetAttributes = (PHidD_GetAttributes)GetProcAddress(hHID, "HidD_GetAttributes");
	HidD_SetFeature = (PHidD_SetFeature)GetProcAddress(hHID, "HidD_SetFeature");
	HidD_GetFeature = (PHidD_GetFeature)GetProcAddress(hHID, "HidD_GetFeature");

	if (!HidD_GetProductString
		|| !HidD_GetAttributes
		|| !HidD_GetHidGuid
		|| !HidD_SetFeature
		|| !HidD_GetFeature) {
		printf("Couldn't find one or more HID entry points\n");
		return;
	}
}

int Open_Device(void) {
	HDEVINFO							DeviceInfoSet;
	GUID								InterfaceClassGuid;
	SP_DEVICE_INTERFACE_DATA			DeviceInterfaceData;
	PSP_DEVICE_INTERFACE_DETAIL_DATA	pDeviceInterfaceDetailData;
	HIDD_ATTRIBUTES						Attributes;
	DWORD								Result;
	DWORD								MemberIndex = 0;
	DWORD								Required;

	//Validar si se "cargó" la biblioteca (DLL)
	if (!hHID)
		return (0);

	//Obtener el Globally Unique Identifier (GUID) para dispositivos HID
	HidD_GetHidGuid(&InterfaceClassGuid);
	//Sacarle a Windows la información sobre todos los dispositivos HID instalados y activos en el sistema
	// ... almacenar esta información en una estructura de datos de tipo HDEVINFO
	DeviceInfoSet = SetupDiGetClassDevs(&InterfaceClassGuid, NULL, NULL, DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);
	if (DeviceInfoSet == INVALID_HANDLE_VALUE)
		return (0);

	//Obtener la interfaz de comunicación con cada uno de los dispositivos para preguntarles información específica
	DeviceInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
	while (!HIDDeviceFound) {
		// ... utilizando la variable MemberIndex ir preguntando dispositivo por dispositivo ...
		moreHIDDevices = SetupDiEnumDeviceInterfaces(DeviceInfoSet, NULL, &InterfaceClassGuid,
			MemberIndex, &DeviceInterfaceData);
		if (!moreHIDDevices) {
			// ... si llegamos al fin de la lista y no encontramos al dispositivo ==> terminar y marcar error
			SetupDiDestroyDeviceInfoList(DeviceInfoSet);
			return (0); //No more devices found
		}
		else {
			//Necesitamos preguntar, a través de la interfaz, el PATH del dispositivo, para eso ...
			// ... primero vamos a ver cuántos caracteres se requieren (Required)
			Result = SetupDiGetDeviceInterfaceDetail(DeviceInfoSet, &DeviceInterfaceData, NULL, 0, &Required, NULL);
			pDeviceInterfaceDetailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(Required);
			if (pDeviceInterfaceDetailData == NULL) {
				printf("Error en SetupDiGetDeviceInterfaceDetail\n");
				return (0);
			}
			//Ahora si, ya que el "buffer" fue preparado (pDeviceInterfaceDetailData{DevicePath}), vamos a preguntar PATH
			pDeviceInterfaceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
			Result = SetupDiGetDeviceInterfaceDetail(DeviceInfoSet, &DeviceInterfaceData, pDeviceInterfaceDetailData,
				Required, NULL, NULL);
			if (!Result) {
				printf("Error en SetupDiGetDeviceInterfaceDetail\n");
				free(pDeviceInterfaceDetailData);
				return(0);
			}
			//Para este momento ya sabemos el PATH del dispositivo, ahora hay que preguntarle ...
			// ... su VID y PID, para ver si es con quien nos interesa comunicarnos
			printf("Found? ==> ");
			printf("Device: %s\n", pDeviceInterfaceDetailData->DevicePath);

			//Obtener un "handle" al dispositivo
			DeviceHandle = CreateFile(pDeviceInterfaceDetailData->DevicePath,
				GENERIC_READ | GENERIC_WRITE,
				FILE_SHARE_READ | FILE_SHARE_WRITE,
				(LPSECURITY_ATTRIBUTES)NULL,
				OPEN_EXISTING,
				0,
				NULL);

			if (DeviceHandle == INVALID_HANDLE_VALUE) {
				printf("¡¡¡Error en el CreateFile!!!\n");
			}
			else {
				//Preguntar por los atributos del dispositivo
				Attributes.Size = sizeof(Attributes);
				Result = HidD_GetAttributes(DeviceHandle, &Attributes);
				if (!Result) {
					printf("Error en HIdD_GetAttributes\n");
					CloseHandle(DeviceHandle);
					free(pDeviceInterfaceDetailData);
					return(0);
				}
				//Analizar los atributos del dispositivo para verificar el VID y PID
				printf("MemberIndex=%d,VID=%04x,PID=%04x\n", MemberIndex, Attributes.VendorID, Attributes.ProductID);
				if ((Attributes.VendorID == RICH_VENDOR_ID) && (Attributes.ProductID == RICH_USBHID_GENIO_ID)) {
					printf("USB/HID GenIO ==> ");
					printf("Device: %s\n", pDeviceInterfaceDetailData->DevicePath);
					HIDDeviceFound = TRUE;
				}
				else
					CloseHandle(DeviceHandle);
			}
			MemberIndex++;
			free(pDeviceInterfaceDetailData);
			if (HIDDeviceFound) {
				printf("Dispositivo HID solicitado ... ¡¡¡localizado!!!, presione <ENTER>\n");
				getchar();
			}
		}
	}
	return(1);
}

void Close_Device(void) {
	if (DeviceHandle != NULL) {
		CloseHandle(DeviceHandle);
		DeviceHandle = NULL;
	}
}

int Touch_Device(void) {
	
	DWORD BytesRead = 0;
	DWORD BytesWritten = 0;
	unsigned char reporteEntrada[INPUT_REPORT_SIZE + 1];
	unsigned char reporteSalida[OUTPUT_REPORT_SIZE + 1];
	int status = 0;
	static unsigned char dato = 0x01;
	static unsigned char numLED = 1;
	int* dato2 = NULL;
	int d1, d2, respuesta;
	int* d1ptr, * d2ptr, * rsptptr;
	float df1, df2, respuestaf;
	float* df1ptr, * df2ptr, * rsptfptr;
	
	for (int i = 0; i < 10; i++) {
		for (int h = 0; h < 10; h++) {

			matrix1[i][h] = h+1.1+i;
			matrix2[i][h] = h+1.1+i;
			matrix3[i][h] = 0.0;
		}
	}
	
	if (DeviceHandle == NULL)	//Validar que haya comunicacion con el dispositivo
		return 0;
	char opcion;
	char opcionled;
	char apagen;
	char numer1;
	char numer2;
		printf("\n");
	printf("Bienvenido, que deseas hacer? \n");
	printf("1.- Modificar  el  estado  de  algun  LED \n");
	printf("2.- Leer  el  estado  de  los  switches \n");
	printf("3.- Leer las matrículas de los programadores del  firmware \n");
	printf("4.- Multiplicacion flotante \n");
	printf("5.- Multiplicacion matrices \n");
	scanf_s(" %c", &opcion, 1);

	switch (opcion) {

	case '1':
		printf("Ingresa el led que deseas apagar o encender");
		scanf_s(" %c", &opcionled, 1);
		int mod;
		switch (opcionled) {

		case '1':

			printf("\n 1.-Encender \n 2.- Apagar \n ");
			scanf_s(" %c", &apagen, 1);
			if (apagen == '1') {
				reporteSalida[0] = 0x00;
				reporteSalida[1] = 0x01;
				reporteSalida[2] = 1;
				status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			}
			else if (apagen == '2') {
				reporteSalida[0] = 0x00;
				reporteSalida[1] = 0x01;
				reporteSalida[2] = 0;
				status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			}
			else {
				printf("opcion invalida");
			}

			break;

		case '2':
			printf("\n 1.-Encender \n 2.- Apagar \n ");
			scanf_s(" %c", &apagen, 1);
			if (apagen == '1') {
				reporteSalida[0] = 0x00;
				reporteSalida[1] = 0x02;
				reporteSalida[2] = 1;
				status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			}
			else if (apagen == '2') {
				reporteSalida[0] = 0x00;
				reporteSalida[1] = 0x02;
				reporteSalida[2] = 0;
				status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			}
			else {
				printf("opcion invalida");
			}
			break;

		case '3':
			printf("\n 1.-Encender \n 2.- Apagar \n ");
			scanf_s(" %c", &apagen, 1);
			if (apagen == '1') {
				reporteSalida[0] = 0x00;
				reporteSalida[1] = 0x03;
				reporteSalida[2] = 1;
				status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			}
			else if (apagen == '2') {
				reporteSalida[0] = 0x00;
				reporteSalida[1] = 0x03;
				reporteSalida[2] = 0;
				status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			}
			else {
				printf("opcion invalida");
			}
		default:

			printf("opcion invalida");

		}

		break;

	case '2':
		printf("Leyendo estado de los switches ...\n");
		reporteSalida[0] = 0x00;	//Dummy
		reporteSalida[1] = 0x81;	//Comando ... leer "push-buttons"
		reporteSalida[2] = 0;

		status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
		if (!status)
			printf("Error en el WriteFile %d %d\n", GetLastError(), BytesWritten);
		else
			printf("Escritos %d\n", BytesWritten);
		memset(&reporteEntrada, 0, INPUT_REPORT_SIZE + 1);
		status = ReadFile(DeviceHandle, reporteEntrada, INPUT_REPORT_SIZE + 1, &BytesRead, NULL);
		dato2 = (int*)&reporteEntrada[2];
		if (!status)
			printf("Error en el ReadFile: %d\n", GetLastError());
		else
			printf("Buffer: %02X %02X %02X\n", (unsigned char)reporteEntrada[0],
				(unsigned char)reporteEntrada[1],
				(unsigned char)reporteEntrada[2]);



		break;
	case '3':
		reporteSalida[0] = 0x00;	//Dummy
		reporteSalida[1] = 0x82;	//Comando ... leer el nombre del programador del firmware
		reporteSalida[2] = 0;
		status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
		memset(&reporteEntrada, 0, INPUT_REPORT_SIZE + 1);
		status = ReadFile(DeviceHandle, reporteEntrada, INPUT_REPORT_SIZE + 1, &BytesRead, NULL);
		if (!status)
			printf("Error en el ReadFile: %d\n", GetLastError());
		else
			printf("Buffer: %02X %02X %s %s \n", (unsigned char)reporteEntrada[0],
				(unsigned char)reporteEntrada[1],
				&reporteEntrada[2],
				&reporteEntrada[12]);
		break;
	case '4':

		printf("Ingresa el primer numero: \n");
		scanf_s(" %f", &df1);

		printf("Ingresa el segundo numero: \n");

		scanf_s(" %f", &df2);

		printf("Voy a multiplicar %f x %f = ", df1, df2);
		reporteSalida[0] = 0x00;	//Dummy
		reporteSalida[1] = 0x22;	//multiplicacion flotante
		df1ptr = (float*)&reporteSalida[2];
		*df1ptr = df1;
		df2ptr = (float*)&reporteSalida[6];
		*df2ptr = df2;
		status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
		memset(&reporteEntrada, 0, INPUT_REPORT_SIZE + 1);
		status = ReadFile(DeviceHandle, reporteEntrada, INPUT_REPORT_SIZE + 1, &BytesRead, NULL);
		rsptfptr = (float*)&reporteEntrada[2];
		respuestaf = *rsptfptr;
		printf("%f\n", respuestaf);
	case '5':
		printf("Escribe el nombre del archivo que vayas a leer");
			
		for (int h = 0; h < 10; h++) {
			printf("Matriz 1");
			reporteSalida[0] = 0x00;
			reporteSalida[1] = 0x14; 
			reporteSalida[2] = h; 

			//enviar la matriz
			for (int i = 0; i < 10; i++) {
				df1ptr = (float*)&reporteSalida[i * 4 + 3];
				*df1ptr = matrix1[h][i];
			}
			status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);

			if (!status)
				printf("Error en la rutina de  WriteFile %d %d \n", GetLastError(), BytesWritten);
			else
				printf("Se han enviado %d bytes al dispositivo \n ", BytesWritten);
			}
		for (int h = 0; h < 10; h++) {
			printf("Matriz 2");
			reporteSalida[0] = 0x00;
			reporteSalida[1] = 0x15; 
			reporteSalida[2] = h; 

			//enviar la matriz
			for (int i = 0; i < 10; i++) {
				df1ptr = (float*)&reporteSalida[i * 4 + 3];
				*df1ptr = matrix1[h][i];
			}
			status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);

			if (!status)
				printf("Error en la rutina de  WriteFile %d %d \n", GetLastError(), BytesWritten);
			else
				printf("Se han enviado %d bytes al dispositivo \n ", BytesWritten);
		}

		reporteSalida[0] = 0x00;
		reporteSalida[1] = 0x16; //multiplica las matrices
		reporteSalida[0] = 0x00;
		status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);

		if (!status)
			printf("Error en el WriteFile %d %d \n", GetLastError(), BytesWritten);
		else
			printf("Se enviaron %d bytes al dispositivo \n ", BytesWritten);

		printf("Matriz Resultante\n");

		for (int i = 0; i < 10; i++) {
			reporteSalida[0] = 0x00;
			reporteSalida[1] = 0xFF;
			reporteSalida[2] = i;

			status = WriteFile(DeviceHandle, reporteSalida, OUTPUT_REPORT_SIZE + 1, &BytesWritten, NULL);
			memset(&reporteEntrada, 0, INPUT_REPORT_SIZE + 1);
			status = ReadFile(DeviceHandle, reporteEntrada, INPUT_REPORT_SIZE + 1, &BytesRead, NULL);

			for (int i = 0; i < 10; i++) {

				rsptfptr = (float*)&reporteEntrada[i * 4 + 3];
				printf("%.2f  ", *rsptfptr);
			}
			printf("\n");
		}
		break;
	default:
		printf("opcion invalida");
	}
	return status;
}
void main() {
	Load_HID_Library();
	if (Open_Device()) {
		printf("Vamos bien\n");
		
		while ((!_kbhit())
			&& (!terminaAbruptaEInstantaneamenteElPrograma)) {
			
			Touch_Device();
			Sleep(500);
		}
	} else {
		printf(">:(\n");
	}
	Close_Device();
}