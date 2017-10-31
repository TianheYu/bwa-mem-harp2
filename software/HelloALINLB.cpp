#include <aalsdk/AALTypes.h>
#include <aalsdk/Runtime.h>
#include <aalsdk/AALLoggerExtern.h>

#include <aalsdk/service/IALIAFU.h>
#include "IMPF.h"

#include <string.h>

#include <math.h>

//****************************************************************************
// UN-COMMENT appropriate #define in order to enable either Hardware or ASE.
//    DEFAULT is to use Software Simulation.
//****************************************************************************
#define  HWAFU
//#define  ASEAFU

using namespace std;
using namespace AAL;

// Convenience macros for printing messages and errors.
#ifdef MSG
# undef MSG
#endif // MSG
#define MSG(x) std::cout << __AAL_SHORT_FILE__ << ':' << __LINE__ << ':' << __AAL_FUNC__ << "() : " << x << std::endl
#ifdef ERR
# undef ERR
#endif // ERR
#define ERR(x) std::cerr << __AAL_SHORT_FILE__ << ':' << __LINE__ << ':' << __AAL_FUNC__ << "() **Error : " << x << std::endl

// Print/don't print the event ID's entered in the event handlers.
#if 1
# define EVENT_CASE(x) case x : MSG(#x);
#else
# define EVENT_CASE(x) case x :
#endif

#ifndef CL
# define CL(x)                     ((x) * 64)
#endif // CL
#ifndef LOG2_CL
# define LOG2_CL                   6
#endif // LOG2_CL
#ifndef MB
# define MB(x)                     ((x) * 1024 * 1024)
#endif // MB
#ifndef KB
# define KB(x)                     ((x) * 1024)
#endif // MB
#define LPBK1_BUFFER_SIZE        CL(1)

#define LPBK1_DSM_SIZE           MB(4)



// Allocate Workspaces needed. ASE runs more slowly and we want to watch the transfers,
//   so have fewer of them.
#define LB_BUFFER_SIZE MB(3080)
#define BWT_ref    MB(3072)
#define CNT_table  MB(1)
#define BWT_input  MB(1)
#define BWT_output MB(1)

unsigned int *SPL_BWT_ref;
unsigned int *SPL_CNT_table;
unsigned int *read_size;
unsigned int *handshake;
unsigned long int *SPL_BWT_input;
unsigned long int *SPL_BWT_output;
unsigned long int *SPL_input_base;
unsigned long int *SPL_output_base;

// define bwa_top //
extern "C"
{
	int top_main(int argc, char *argv[]);
}
// LPBK1_BUFFER_SIZE is size in cachelines that are copied
#define LPBK1_BUFFER_SIZE        CL(1)
// LPBK1_BUFFER_ALLOCATION_SIZE is the amount of space that needs to
//   be allocated due an optimization of the NLB AFU to operate on
//   2 MiB buffer boundaries. Note that the way to get 2 MiB alignment
//   is to allocate 2 MiB.
// NOTE:
//   2 MiB alignment is not a general requirement -- it is NLB-specific
#define LPBK1_BUFFER_ALLOCATION_SIZE MB(2)

#define LPBK1_DSM_SIZE           MB(4)
#define CSR_SRC_ADDR             0x0120
#define CSR_DST_ADDR             0x0128
#define CSR_CTL                  0x0138
#define CSR_CFG                  0x0140
#define CSR_NUM_LINES            0x0130
#define DSM_STATUS_TEST_COMPLETE 0x40
#define CSR_AFU_DSM_BASEL        0x0110
#define CSR_AFU_DSM_BASEH        0x0114
#define NLB_TEST_MODE_PCIE0		0x2000



/// @brief   Since this is a simple application, our App class implements both the IRuntimeClient and IServiceClient
///           interfaces.  Since some of the methods will be redundant for a single object, they will be ignored.
///
class HelloALINLBApp: public CAASBase, public IRuntimeClient, public IServiceClient
{
public:

   HelloALINLBApp();
   ~HelloALINLBApp();

   btInt run(int argc, char *argv[]);    ///< Return 0 if success

   // <begin IServiceClient interface>
   void serviceAllocated(IBase *pServiceBase,
                         TransactionID const &rTranID);

   void serviceAllocateFailed(const IEvent &rEvent);

   void serviceReleased(const AAL::TransactionID&);
   void serviceReleaseRequest(IBase *pServiceBase, const IEvent &rEvent);
   void serviceReleaseFailed(const AAL::IEvent&);

   void serviceEvent(const IEvent &rEvent);
   // <end IServiceClient interface>

   // <begin IRuntimeClient interface>
   void runtimeCreateOrGetProxyFailed(IEvent const &rEvent){};    // Not Used

   void runtimeStarted(IRuntime            *pRuntime,
                       const NamedValueSet &rConfigParms);

   void runtimeStopped(IRuntime *pRuntime);

   void runtimeStartFailed(const IEvent &rEvent);

   void runtimeStopFailed(const IEvent &rEvent);

   void runtimeAllocateServiceFailed( IEvent const &rEvent);

   void runtimeAllocateServiceSucceeded(IBase               *pClient,
                                        TransactionID const &rTranID);

   void runtimeEvent(const IEvent &rEvent);

   btBool isOK()  {return m_bIsOK;}

   // <end IRuntimeClient interface>
protected:
   Runtime        m_Runtime;           ///< AAL Runtime
   IBase         *m_pALIAFU_AALService;       ///< The generic AAL Service interface for the AFU.
   IALIBuffer    *m_pALIBufferService; ///< Pointer to Buffer Service
   IALIMMIO      *m_pALIMMIOService;   ///< Pointer to MMIO Service
   IALIReset     *m_pALIResetService;  ///< Pointer to AFU Reset Service
   CSemaphore     m_Sem;               ///< For synchronizing with the AAL runtime.
   btInt          m_Result;            ///< Returned result value; 0 if success
   TransactionID  m_ALIAFUTranID;           ///< TransactionID used for service allocation

											// VTP service-related information
   IBase         *m_pVTP_AALService;        ///< The generic AAL Service interface for the VTP.
   IMPFVTP       *m_pVTPService;            ///< Pointer to VTP buffer service
   btCSROffset    m_VTPDFHOffset;           ///< VTP DFH offset
   TransactionID  m_VTPTranID;              ///< TransactionID used for service allocation

   // Workspace info
   btVirtAddr     m_DSMVirt;        ///< DSM workspace virtual address.
   btPhysAddr     m_DSMPhys;        ///< DSM workspace physical address.
   btWSSize       m_DSMSize;        ///< DSM workspace size in bytes.
   btVirtAddr     m_InputVirt;      ///< Input workspace virtual address.
   btPhysAddr     m_InputPhys;      ///< Input workspace physical address.
   btWSSize       m_InputSize;      ///< Input workspace size in bytes.
   btVirtAddr     m_OutputVirt;     ///< Output workspace virtual address.
   btPhysAddr     m_OutputPhys;     ///< Output workspace physical address.
   btWSSize       m_OutputSize;     ///< Output workspace size in bytes.
};

///////////////////////////////////////////////////////////////////////////////
///
///  Implementation
///
///////////////////////////////////////////////////////////////////////////////

/// @brief   Constructor registers this objects client interfaces and starts
///          the AAL Runtime. The member m_bisOK is used to indicate an error.
///
HelloALINLBApp::HelloALINLBApp() :
   m_Runtime(this),
   m_pALIAFU_AALService(NULL),
   m_pALIBufferService(NULL),
   m_pALIMMIOService(NULL),
   m_pALIResetService(NULL),
   m_Result(0),
	m_pVTP_AALService(NULL),
	m_pVTPService(NULL),
   m_VTPDFHOffset(-1),
   m_DSMVirt(NULL),
   m_DSMPhys(0),
   m_DSMSize(0),
   m_InputVirt(NULL),
   m_InputPhys(0),
   m_InputSize(0),
   m_OutputVirt(NULL),
   m_OutputPhys(0),
   m_OutputSize(0),
	m_ALIAFUTranID(),
	m_VTPTranID()
{
   // Register our Client side interfaces so that the Service can acquire them.
   //   SetInterface() is inherited from CAASBase
   SetInterface(iidServiceClient, dynamic_cast<IServiceClient *>(this));
   SetInterface(iidRuntimeClient, dynamic_cast<IRuntimeClient *>(this));

   // Initialize our internal semaphore
   m_Sem.Create(0, 1);

   // Start the AAL Runtime, setting any startup options via a NamedValueSet

   // Using Hardware Services requires the Remote Resource Manager Broker Service
   //  Note that this could also be accomplished by setting the environment variable
   //   AALRUNTIME_CONFIG_BROKER_SERVICE to librrmbroker
   NamedValueSet configArgs;
   NamedValueSet configRecord;

#if defined( HWAFU )
   // Specify that the remote resource manager is to be used.
   configRecord.Add(AALRUNTIME_CONFIG_BROKER_SERVICE, "librrmbroker");
   configArgs.Add(AALRUNTIME_CONFIG_RECORD, &configRecord);
#endif

   // Start the Runtime and wait for the callback by sitting on the semaphore.
   //   the runtimeStarted() or runtimeStartFailed() callbacks should set m_bIsOK appropriately.
   if(!m_Runtime.start(configArgs)){
	   m_bIsOK = false;
      return;
   }
   m_Sem.Wait();
   m_bIsOK = true;
}

/// @brief   Destructor
///
HelloALINLBApp::~HelloALINLBApp()
{
   m_Sem.Destroy();
}

/// @brief   run() is called from main performs the following:
///             - Allocate the appropriate ALI Service depending
///               on whether a hardware, ASE or software implementation is desired.
///             - Allocates the necessary buffers to be used by the NLB AFU algorithm
///             - Executes the NLB algorithm
///             - Cleans up.
///
btInt HelloALINLBApp::run(int argc, char *argv[])
{
   cout <<"========================"<<endl;
   cout <<"= Hello ALI NLB Sample ="<<endl;
   cout <<"========================"<<endl;

   // Request the Servcie we are interested in.

   // NOTE: This example is bypassing the Resource Manager's configuration record lookup
   //  mechanism.  Since the Resource Manager Implementation is a sample, it is subject to change.
   //  This example does illustrate the utility of having different implementations of a service all
   //  readily available and bound at run-time.
   NamedValueSet Manifest;
   NamedValueSet ConfigRecord;
   NamedValueSet featureFilter;
   btcString sGUID = MPF_VTP_BBB_GUID;

#if defined( HWAFU )                /* Use FPGA hardware */
   // Service Library to use
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libALI"); //"libHWALIAFU");

   // the AFUID to be passed to the Resource Manager. It will be used to locate the appropriate device.

   ConfigRecord.Add(keyRegAFU_ID,"04242017-DEAD-BEEF-DEAD-BEEF01234567");
//ConfigRecord.Add(keyRegAFU_ID,"C000C966-0D82-4272-9AEF-FE5F84570612");

   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_AIA_NAME, "libaia");

   #elif defined ( ASEAFU )         /* Use ASE based RTL simulation */
   Manifest.Add(keyRegHandle, 20);

   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libASEALIAFU");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE,true);

   #else                            /* default is Software Simulator */
#if 0 // NOT CURRRENTLY SUPPORTED
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libSWSimALIAFU");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE,true);
#endif
   return -1;
#endif

   // Add the Config Record to the Manifest describing what we want to allocate
   Manifest.Add(AAL_FACTORY_CREATE_CONFIGRECORD_INCLUDED, &ConfigRecord);

   // in future, everything could be figured out by just giving the service name
   Manifest.Add(AAL_FACTORY_CREATE_SERVICENAME, "Hello ALI NLB");

   MSG("Allocating Service");

   // Allocate the Service and wait for it to complete by sitting on the
   //   semaphore. The serviceAllocated() callback will be called if successful.
   //   If allocation fails the serviceAllocateFailed() should set m_bIsOK appropriately.
   //   (Refer to the serviceAllocated() callback to see how the Service's interfaces
   //    are collected.)
   m_Runtime.allocService(dynamic_cast<IBase *>(this), Manifest, m_ALIAFUTranID);//);
   m_Sem.Wait();
   if(!m_bIsOK){
      ERR("Allocation failed\n");
      goto done_0;
   }

   // Now that we have the Service and have saved the IALIBuffer interface pointer
   //  we can now Allocate the 3 Workspaces used by the NLB algorithm. The buffer allocate
   //  function is synchronous so no need to wait on the semaphore

   // Reuse Manifest and Configrecord for VTP service
   Manifest.Empty();
   ConfigRecord.Empty();

   // Allocate VTP service
   // Service Library to use
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libMPF");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE, true);

   // Add the Config Record to the Manifest describing what we want to allocate
   Manifest.Add(AAL_FACTORY_CREATE_CONFIGRECORD_INCLUDED, &ConfigRecord);

   // the VTPService will reuse the already established interfaces presented by
   // the ALIAFU service
   Manifest.Add(ALIAFU_IBASE_KEY, static_cast<ALIAFU_IBASE_DATATYPE>(m_pALIAFU_AALService));

   // MPFs feature ID, used to find correct features in DFH list
   Manifest.Add(MPF_FEATURE_ID_KEY, static_cast<MPF_FEATURE_ID_DATATYPE>(1));

   // in future, everything could be figured out by just giving the service name
   Manifest.Add(AAL_FACTORY_CREATE_SERVICENAME, "VTP");

   MSG("Allocating VTP Service");

   m_Runtime.allocService(dynamic_cast<IBase *>(this), Manifest, m_VTPTranID);
   m_Sem.Wait();
   if (!m_bIsOK) {
	   ERR("VTP Service allocation failed\n");
	   goto done_0;
   }
   MSG("VTP Service allocated");

   // Device Status Memory (DSM) is a structure defined by the NLB implementation.
   MSG("dsm");
   // User Virtual address of the pointer is returned directly in the function
   if( ali_errnumOK != m_pVTPService->bufferAllocate(LPBK1_DSM_SIZE, &m_DSMVirt)){
      m_bIsOK = false;
      m_Result = -1;
      goto done_1;
   }

   // Save the size and get the IOVA from the User Virtual address. The HW only uses IOVA.
   m_DSMSize = LPBK1_DSM_SIZE;

   MSG("input");
   if (ali_errnumOK != m_pVTPService->bufferAllocate(LB_BUFFER_SIZE, &m_InputVirt)) {
	   m_bIsOK = false;
	   m_Sem.Post(1);
	   m_Result = -1;
	   goto done_2;
   }

   m_InputSize = LB_BUFFER_SIZE;
   
   MSG("output");
   if( ali_errnumOK != m_pVTPService->bufferAllocate(LPBK1_BUFFER_ALLOCATION_SIZE, &m_OutputVirt)){
      m_bIsOK = false;
      m_Sem.Post(1);
      m_Result = -1;
      goto done_3;
   }

   //=============================
   // Now we have the NLB Service
   //   now we can use it
   //=============================
   MSG("Running Test");
   if(true == m_bIsOK){
	   MSG("m_pInput == 0x" << std::hex << (btUnsigned64bitInt)m_InputVirt);
	   MSG("m_pOutput == 0x" << std::hex << (btUnsigned64bitInt)m_OutputVirt);
      // Clear the DSM
      ::memset( m_DSMVirt, 0, m_DSMSize);

      // Initialize the source and destination buffers
      //::memset( m_InputVirt,  0xAF, m_InputSize);  // Input initialized to AFter
	  btVirtAddr         pSource = m_InputVirt;

	  SPL_BWT_ref = (unsigned int *)(pSource);
	  SPL_CNT_table = (unsigned int *)(pSource);
	  handshake = (unsigned int *)(pSource);
	  read_size = (unsigned int *)(pSource);
	  SPL_BWT_input = (unsigned long int *)(pSource);
	  SPL_BWT_output = (unsigned long int *)(pSource);
	  SPL_input_base = (unsigned long int *)(pSource);
	  SPL_output_base = (unsigned long int *)(pSource);

	  SPL_CNT_table += BWT_ref / sizeof(unsigned int);
	  handshake += ((BWT_ref + CNT_table) / sizeof(unsigned int)) - 1;
	  read_size += ((BWT_ref + CNT_table) / sizeof(unsigned int)) - 2;
	  SPL_BWT_input += (BWT_ref + CNT_table) / sizeof(unsigned long int);
	  SPL_input_base += (BWT_ref + CNT_table) / sizeof(unsigned long int);
	  SPL_BWT_output += (BWT_ref + CNT_table + BWT_input) / sizeof(unsigned long int);
	  SPL_output_base += (BWT_ref + CNT_table + BWT_input) / sizeof(unsigned long int);
	  //change here to assign input data space from source pointer
	  

      // Initiate AFU Reset
      m_pALIResetService->afuReset();
	  m_pVTPService->vtpReset();

      // Initiate DSM Reset
      // Set DSM base, high then low
      m_pALIMMIOService->mmioWrite64(CSR_AFU_DSM_BASEL, (btUnsigned64bitInt)(m_DSMVirt)/CL(1));

      // Assert AFU reset
      m_pALIMMIOService->mmioWrite32(CSR_CTL, 0);

      //De-Assert AFU reset
      m_pALIMMIOService->mmioWrite32(CSR_CTL, 1);

      // If ASE, give it some time to catch up

      #if defined ( ASEAFU )
      SleepSec(5);
      #endif

      // Set input workspace address		//here is the working address
      m_pALIMMIOService->mmioWrite64(CSR_SRC_ADDR, (btUnsigned64bitInt)(m_InputVirt) / CL(1));

      // Set output workspace address
      m_pALIMMIOService->mmioWrite64(CSR_DST_ADDR, (btUnsigned64bitInt)(m_OutputVirt) / CL(1));

      // Set the number of cache lines for the test
	  m_pALIMMIOService->mmioWrite32(CSR_NUM_LINES, 16); //LPBK1_BUFFER_SIZE / CL(1));256*32

      // Set the test mode
      m_pALIMMIOService->mmioWrite32(CSR_CFG,0);

      //volatile bt32bitCSR *StatusAddr = (volatile bt32bitCSR *)(m_DSMVirt  + DSM_STATUS_TEST_COMPLETE);	
	  //could be handshake pointer
      // Start the test
      m_pALIMMIOService->mmioWrite32(CSR_CTL, 3);

	  MSG("source physical=" << std::hex << (void *)m_InputPhys <<
		  " source virtual=" << std::hex << (void *)m_InputVirt <<
		  " handshake virtual=" << std::hex << (void *)handshake );

	  MSG("Start virtual: " << std::hex << (void *)SPL_CNT_table <<
			"End Virtual: " << std::hex << (void *)SPL_output_base);
	  int top_val;
	  top_val = top_main(argc, argv);

      // Stop the device
      m_pALIMMIOService->mmioWrite32(CSR_CTL, 7);
   }
   MSG("Done Running Test");

   // Clean-up and return
done_4:
   m_pVTPService->bufferFree(m_OutputVirt);
done_3:
   m_pVTPService->bufferFree(m_InputVirt);
done_2:
   m_pVTPService->bufferFree(m_DSMVirt);

done_1:
   // Freed all three so now Release() the Service through the Services IAALService::Release() method
   (dynamic_ptr<IAALService>(iidService, m_pALIAFU_AALService))->Release(TransactionID());
   m_Sem.Wait();

done_0:
   m_Runtime.stop();
   m_Sem.Wait();

   return m_Result;
}

//=================
//  IServiceClient
//=================

// <begin IServiceClient interface>
void HelloALINLBApp::serviceAllocated(IBase *pServiceBase,
                                      TransactionID const &rTranID)
{
	if (rTranID == m_ALIAFUTranID) {
		// Save the IBase for the Service. Through it we can get any other
		//  interface implemented by the Service
		m_pALIAFU_AALService = pServiceBase;
		ASSERT(NULL != m_pALIAFU_AALService);
		if (NULL == m_pALIAFU_AALService) {
			m_bIsOK = false;
			return;
		}

		// Documentation says HWALIAFU Service publishes
		//    IALIBuffer as subclass interface. Used in Buffer Allocation and Free
		m_pALIBufferService = dynamic_ptr<IALIBuffer>(iidALI_BUFF_Service, pServiceBase);
		ASSERT(NULL != m_pALIBufferService);
		if (NULL == m_pALIBufferService) {
			m_bIsOK = false;
			return;
		}

		// Documentation says HWALIAFU Service publishes
		//    IALIMMIO as subclass interface. Used to set/get MMIO Region
		m_pALIMMIOService = dynamic_ptr<IALIMMIO>(iidALI_MMIO_Service, pServiceBase);
		ASSERT(NULL != m_pALIMMIOService);
		if (NULL == m_pALIMMIOService) {
			m_bIsOK = false;
			return;
		}

		// Documentation says HWALIAFU Service publishes
		//    IALIReset as subclass interface. Used for resetting the AFU
		m_pALIResetService = dynamic_ptr<IALIReset>(iidALI_RSET_Service, pServiceBase);
		ASSERT(NULL != m_pALIResetService);
		if (NULL == m_pALIResetService) {
			m_bIsOK = false;
			return;
		}

		MSG("Service Allocated");
	}
	else if (rTranID == m_VTPTranID) {

		// Save the IBase for the VTP Service.
		m_pVTP_AALService = pServiceBase;
		ASSERT(NULL != m_pVTP_AALService);
		if (NULL == m_pVTP_AALService) {
			m_bIsOK = false;
			return;
		}

		// Documentation says VTP Service publishes
		//    IVTP as subclass interface. Used for allocating shared
		//    buffers that support virtual addresses from AFU
		m_pVTPService = dynamic_ptr<IMPFVTP>(iidMPFVTPService, pServiceBase);
		ASSERT(NULL != m_pVTPService);
		if (NULL == m_pVTPService) {
			m_bIsOK = false;
			return;
		}

		MSG("VTP Service Allocated");
	}
	else
	{
		ERR("Unknown transaction ID encountered on serviceAllocated().");
		m_bIsOK = false;
		return;
	}

   m_Sem.Post(1);
}

void HelloALINLBApp::serviceAllocateFailed(const IEvent &rEvent)
{
   ERR("Failed to allocate Service");
    PrintExceptionDescription(rEvent);
   ++m_Result;                     // Remember the error
   m_bIsOK = false;

   m_Sem.Post(1);
}

 void HelloALINLBApp::serviceReleased(TransactionID const &rTranID)
{
    MSG("Service Released");
   // Unblock Main()
   m_Sem.Post(1);
}

 void HelloALINLBApp::serviceReleaseRequest(IBase *pServiceBase, const IEvent &rEvent)
 {
    MSG("Service unexpected requested back");
    if(NULL != m_pALIAFU_AALService){
       IAALService *pIAALService = dynamic_ptr<IAALService>(iidService, m_pALIAFU_AALService);
       ASSERT(pIAALService);
       pIAALService->Release(TransactionID());
    }
 }


 void HelloALINLBApp::serviceReleaseFailed(const IEvent        &rEvent)
 {
    ERR("Failed to release a Service");
    PrintExceptionDescription(rEvent);
    m_bIsOK = false;
    m_Sem.Post(1);
 }


 void HelloALINLBApp::serviceEvent(const IEvent &rEvent)
{
   ERR("unexpected event 0x" << hex << rEvent.SubClassID());
   // The state machine may or may not stop here. It depends upon what happened.
   // A fatal error implies no more messages and so none of the other Post()
   //    will wake up.
   // OTOH, a notification message will simply print and continue.
}
// <end IServiceClient interface>


 //=================
 //  IRuntimeClient
 //=================

  // <begin IRuntimeClient interface>
 // Because this simple example has one object implementing both IRuntieCLient and IServiceClient
 //   some of these interfaces are redundant. We use the IServiceClient in such cases and ignore
 //   the RuntimeClient equivalent e.g.,. runtimeAllocateServiceSucceeded()

 void HelloALINLBApp::runtimeStarted( IRuntime            *pRuntime,
                                      const NamedValueSet &rConfigParms)
 {
    m_bIsOK = true;
    m_Sem.Post(1);
 }

 void HelloALINLBApp::runtimeStopped(IRuntime *pRuntime)
  {
     MSG("Runtime stopped");
     m_bIsOK = false;
     m_Sem.Post(1);
  }

 void HelloALINLBApp::runtimeStartFailed(const IEvent &rEvent)
 {
    ERR("Runtime start failed");
    PrintExceptionDescription(rEvent);
 }

 void HelloALINLBApp::runtimeStopFailed(const IEvent &rEvent)
 {
     MSG("Runtime stop failed");
     m_bIsOK = false;
     m_Sem.Post(1);
 }

 void HelloALINLBApp::runtimeAllocateServiceFailed( IEvent const &rEvent)
 {
    ERR("Runtime AllocateService failed");
    PrintExceptionDescription(rEvent);
 }

 void HelloALINLBApp::runtimeAllocateServiceSucceeded(IBase *pClient,
                                                     TransactionID const &rTranID)
 {
     MSG("Runtime Allocate Service Succeeded");
 }

 void HelloALINLBApp::runtimeEvent(const IEvent &rEvent)
 {
     MSG("Generic message handler (runtime)");
 }
 // <begin IRuntimeClient interface>

/// @} group HelloALINLB


//=============================================================================
// Name: main
// Description: Entry point to the application
// Inputs: none
// Outputs: none
// Comments: Main initializes the system. The rest of the example is implemented
//           in the object theApp.
//=============================================================================
int main(int argc, char *argv[])
{
   HelloALINLBApp theApp;

   if(!theApp.isOK()){
      ERR("Runtime Failed to Start");
      exit(1);
   }
   btInt Result = theApp.run(argc, argv);

   MSG("Done");
   if (0 == Result) {
	   MSG("======= SUCCESS =======");
   }
   else {
	   MSG("!!!!!!! FAILURE !!!!!!!");
   }
   return Result;
}


