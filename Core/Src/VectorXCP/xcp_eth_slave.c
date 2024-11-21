
#include <stdint.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "lwip/udp.h"
#include "xcp_eth_slave.h"
#include "xcp.h"
#include "xcp_cfg.h"
#include "xcptl_cfg.h"
#include "xcp_measures.h"
#include "patch.h"

//#define XCP_CPUTYPE_BIGENDIAN

#define XCP_UDP_PORT         5555
#define XCP_MAX_CTO_SIZE     256
#define XCP_MAX_DTO_SIZE     1024

#define XCPTL_MAX_CTO_SIZE 252 // must be mod 4
#define XCPTL_TRANSPORT_LAYER_HEADER_SIZE 4

/*-------------------------------------------------------------------------*/
/* Packet Identifiers Server -> Master */
#define PID_RES                           0xFF   /* response packet        */
#define PID_ERR                           0xFE   /* error packet           */
#define PID_EV                            0xFD   /* event packet           */
#define PID_SERV                          0xFC   /* service request packet */


/****************************************************************************/
/* Defaults and checks                                                      */
/****************************************************************************/

/* Check limits of the XCP imnplementation */
#if defined( XCPTL_MAX_CTO_SIZE )
#if ( XCPTL_MAX_CTO_SIZE > 255 )
#error "XCPTL_MAX_CTO_SIZE must be <= 255"
#endif
#if ( XCPTL_MAX_CTO_SIZE < 8 )
#error "XCPTL_MAX_CTO_SIZE must be >= 8"
#endif
#else
#error "Please define XCPTL_CTO_SIZE"
#endif

#if defined( XCPTL_MAX_DTO_SIZE )
#if ( XCPTL_MAX_DTO_SIZE > (XCPTL_MAX_SEGMENT_SIZE-4) )
#error "XCPTL_MAX_DTO_SIZE too large"
#endif
#if ( XCPTL_MAX_DTO_SIZE < 8 )
#error "XCPTL_MAX_DTO_SIZE must be >= 8"
#endif
#else
#error "Please define XCPTL_DTO_SIZE"
#endif

/* Max. size of an object referenced by an ODT entry XCP_MAX_ODT_ENTRY_SIZE may be limited  */
#if defined ( XCP_MAX_ODT_ENTRY_SIZE )
#if ( XCP_MAX_DTO_ENTRY_SIZE > 255 )
#error "XCP_MAX_ODT_ENTRY_SIZE too large"
#endif
#else
#define XCP_MAX_ODT_ENTRY_SIZE 248 // mod 4 = 0 to optimize DAQ copy granularity
#endif

/* Check XCP_DAQ_MEM_SIZE */
#if defined ( XCP_DAQ_MEM_SIZE )
#if ( XCP_DAQ_MEM_SIZE > 0xFFFFFFFF )
#error "XCP_DAQ_MEM_SIZE must be <= 0xFFFFFFFF"
#endif
#else
#error "Please define XCP_DAQ_MEM_SIZE"
#endif

// Dynamic addressing (ext=1, addr=(event<<16)|offset requires transport layer mode XCPTL_QUEUED_CRM
#if defined(XCP_ENABLE_DYN_ADDRESSING) && !defined(XCPTL_QUEUED_CRM)
#error "Dynamic address format (ext=1) requires XCPTL_QUEUED_CRM!"
#endif


/****************************************************************************/
/* XCP Packet                                                */
/****************************************************************************/

typedef union {
    /* There might be a loss of up to 3 bytes. */
    uint8_t  b[((XCPTL_MAX_CTO_SIZE + 3) & 0xFFC)];
    uint16_t w[((XCPTL_MAX_CTO_SIZE + 3) & 0xFFC) / 2];
    uint32_t dw[((XCPTL_MAX_CTO_SIZE + 3) & 0xFFC) / 4];
} tXcpCto;

/****************************************************************************/
/* Protocol layer data                                                      */
/****************************************************************************/

typedef struct {
    uint16_t dlc;    // lenght
    uint16_t ctr;    // message counter
    tXcpCto Cro;  // message data
} tXcpMessageFromMaster;
tXcpMessageFromMaster *g_pXcpMessageFromMaster = NULL;

#define CRO                       (g_pXcpMessageFromMaster->Cro)
#define CRO_LEN                   (g_pXcpMessageFromMaster->dlc)
#define CRO_BYTE(x)               (g_pXcpMessageFromMaster->Cro.b[x])
#define CRO_WORD(x)               (g_pXcpMessageFromMaster->Cro.w[x])
#define CRO_DWORD(x)              (g_pXcpMessageFromMaster->Cro.dw[x])


typedef struct {
    uint16_t dlc;
    uint16_t ctr;
    uint8_t packet[XCPTL_MAX_CTO_SIZE];
} tXcpCtoMessage;

typedef struct 
{
    uint16_t SessionStatus;    
    uint8_t CrmLen;                        /* RES,ERR message length */
    tXcpCto Crm;                           /* RES,ERR message buffer */

    uint32_t MtaAddr;    
    struct udp_pcb* connection;
    uint8_t connected;
    ip_addr_t remote_addr;
    uint16_t remote_port;
    uint8_t rx_buffer[XCP_MAX_CTO_SIZE];
    uint16_t rx_length;
    uint16_t lastCtr;
} XcpUdpState;

static XcpUdpState g_xcp_udp_state = {.SessionStatus=0, .CrmLen=0, .Crm={0}, .MtaAddr=0, .connection=NULL, .lastCtr=0};

#define CRM                       (g_xcp_udp_state.Crm)
#define CRM_LEN                   (g_xcp_udp_state.CrmLen)
#define CRM_BYTE(x)               (g_xcp_udp_state.Crm.b[x])
#define CRM_WORD(x)               (g_xcp_udp_state.Crm.w[x])
#define CRM_DWORD(x)              (g_xcp_udp_state.Crm.dw[x])



// Extended Connect Response
static void XcpSendResponse() 
{
    // Build XCP CTO message (ctr+dlc+packet)
    tXcpCtoMessage msg;
    msg.dlc = (uint16_t)g_xcp_udp_state.CrmLen;
    msg.ctr = g_xcp_udp_state.lastCtr++;
    memcpy(msg.packet, &g_xcp_udp_state.Crm, g_xcp_udp_state.CrmLen);
    

    uint16_t buf_size = msg.dlc + XCPTL_TRANSPORT_LAYER_HEADER_SIZE;

    struct pbuf* response_buf = pbuf_alloc(PBUF_TRANSPORT, buf_size, PBUF_RAM);
    memcpy(response_buf->payload, (void*)&msg, buf_size);
    response_buf->len = buf_size;

    printf("sending response (size will be %d bytes)\n", buf_size);

    udp_sendto(g_xcp_udp_state.connection, response_buf, 
               &g_xcp_udp_state.remote_addr, 
               g_xcp_udp_state.remote_port);
    
    pbuf_free(response_buf);
}


static void xcp_process_udp_command(uint8_t* data) 
{
    #define return_xcp_command_error(err) {  g_xcp_udp_state.CrmLen = 2; CRM_CMD=PID_ERR; CRM_ERR=(err); XcpSendResponse(); return; }

    g_pXcpMessageFromMaster = (tXcpMessageFromMaster*)data;
    printf("tXcpCtoMessage.dlc=%d, tXcpCtoMessage.packet[0]=%02x\n", g_pXcpMessageFromMaster->dlc, CRO_CMD);
  
    // Check for extended connect request format
    if(CRO_LEN==CRO_CONNECT_LEN && CRO_CMD==CC_CONNECT)   // Connect command
    {
        printf("XCP connect request received:\n");          
        g_xcp_udp_state.connected = 1;
        g_xcp_udp_state.SessionStatus = (uint16_t)(SS_INITIALIZED | SS_STARTED | SS_CONNECTED | SS_LEGACY_MODE);    
        // Prepare response
        CRM_CMD = PID_RES; /* Response, no error */
        CRM_ERR = 0;
        g_xcp_udp_state.CrmLen = CRM_CONNECT_LEN;
        CRM_CONNECT_TRANSPORT_VERSION = (uint8_t)( (uint16_t)XCP_TRANSPORT_LAYER_VERSION >> 8 ); /* Major versions of the XCP Protocol Layer and Transport Layer Specifications. */
        CRM_CONNECT_PROTOCOL_VERSION =  (uint8_t)( (uint16_t)XCP_PROTOCOL_LAYER_VERSION >> 8 );
        CRM_CONNECT_MAX_CTO_SIZE = XCPTL_MAX_CTO_SIZE;
        CRM_CONNECT_MAX_DTO_SIZE = XCPTL_MAX_DTO_SIZE;
        CRM_CONNECT_RESOURCE = 0; //RM_DAQ;       /* RM_DAQ = Data Acquisition supported */
        CRM_CONNECT_COMM_BASIC = CMB_OPTIONAL;
        #if defined ( XCP_CPUTYPE_BIGENDIAN )
            CRM_CONNECT_COMM_BASIC |= (uint8_t)PI_MOTOROLA;
        #endif        
        XcpSendResponse();
    }
    else if( CRO_LEN==CRO_GET_STATUS_LEN && CRO_CMD==CC_GET_STATUS  )
    {
        printf("CC_GET_STATUS request received\n");
        // Prepare response
        CRM_CMD = PID_RES; /* Response, no error */
        CRM_ERR = 0;
        g_xcp_udp_state.CrmLen = CRM_GET_STATUS_LEN;
        CRM_GET_STATUS_STATUS = (uint8_t)(g_xcp_udp_state.SessionStatus&0xFF);
        CRM_GET_STATUS_PROTECTION = 0;
        CRM_GET_STATUS_CONFIG_ID = 0; /* Session configuration ID not available. */
        XcpSendResponse();
    }
    else if( CRO_LEN==CRO_DISCONNECT_LEN && CRO_CMD==CC_DISCONNECT  )
    {
        printf("XCP disconnect request received\n");        
        g_xcp_udp_state.connected = 0;
        g_xcp_udp_state.lastCtr=0;
        g_xcp_udp_state.SessionStatus &= (uint16_t)(~SS_CONNECTED);
    }
    else if( CRO_LEN==CRO_GET_COMM_MODE_INFO_LEN && CRO_CMD==CC_GET_COMM_MODE_INFO )
    {
        printf("GET_COMM_MODE_INFO request received\n");
        // Prepare response
        CRM_CMD = PID_RES; /* Response, no error */
        CRM_ERR = 0;
        g_xcp_udp_state.CrmLen = CRM_GET_COMM_MODE_INFO_LEN;
        CRM_GET_COMM_MODE_INFO_COMM_OPTIONAL  = 0x01;
        CRM_GET_COMM_MODE_INFO_MAX_BS         = 0x01;
        CRM_GET_COMM_MODE_INFO_MIN_ST         = 0x00;
        CRM_GET_COMM_MODE_INFO_QUEUE_SIZE     = 0x01;
        CRM_GET_COMM_MODE_INFO_DRIVER_VERSION = 0x10;
        XcpSendResponse();
    }
    else if( CRO_LEN==CRO_SET_MTA_LEN && CRO_CMD==CC_SET_MTA  )
    {
        g_xcp_udp_state.MtaAddr = CRO_SET_MTA_ADDR;
        printf("CC_SET_MTA request received with addr=%d\n", g_xcp_udp_state.MtaAddr );
        // Prepare response
        CRM_CMD = PID_RES; /* Response, no error */
        CRM_ERR = 0;
        g_xcp_udp_state.CrmLen = CRM_SET_MTA_LEN;
        XcpSendResponse();
    }
    else if( CRO_LEN==CRO_SHORT_UPLOAD_LEN && CRO_CMD==CC_SHORT_UPLOAD  )
    {
        uint8_t size = CRO_SHORT_UPLOAD_SIZE;
        if (size > CRM_SHORT_UPLOAD_MAX_SIZE)   return_xcp_command_error(CRC_OUT_OF_RANGE);        
        g_xcp_udp_state.MtaAddr = CRO_SHORT_UPLOAD_ADDR;

        printf("MTA address requested is: %04x\n", g_xcp_udp_state.MtaAddr);        
        if( size != XcpReadMta(g_xcp_udp_state.MtaAddr, size, CRM_SHORT_UPLOAD_DATA) ) return_xcp_command_error(CRC_ACCESS_DENIED);
        g_xcp_udp_state.CrmLen = (uint8_t)(CRM_SHORT_UPLOAD_LEN+size);
        XcpSendResponse();
    }
    else 
    {
        // Process other XCP commands
        printf("Command received: ");
        printf("tXcpCtoMessage.dlc=%d, tXcpCtoMessage.packet[0]=%02x\n", g_pXcpMessageFromMaster->dlc,CRO_CMD);
        return_xcp_command_error(CRC_CMD_UNKNOWN);
    }

}


// UDP Receive Callback
static void xcp_udp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) 
{
    if (p == NULL) return;

    g_xcp_udp_state.rx_length = p->len;
    pbuf_copy_partial(p, g_xcp_udp_state.rx_buffer, p->len, 0);
    
    // Store remote address and port for response
    ip_addr_copy(g_xcp_udp_state.remote_addr, *addr);
    g_xcp_udp_state.remote_port = port;

    printf("                          ----\n");
    printf("                         | %02d |\n", g_xcp_udp_state.lastCtr);
    printf("                          ----\n");


    // Distant IP address 
    char ip_addr_str[16];
    ip4addr_ntoa_r(addr, ip_addr_str, sizeof(ip_addr_str));
    printf("udp receive from %s: ", ip_addr_str );
    for (int16_t i=0; i<g_xcp_udp_state.rx_length; i++)
    {
        printf("%02X", g_xcp_udp_state.rx_buffer[i]);
    }
    puts("");

    xcp_process_udp_command(g_xcp_udp_state.rx_buffer);
    
    pbuf_free(p);
}

// Initialize UDP XCP Slave
HAL_StatusTypeDef XCP_Udp_Slave_Init(void) 
{
    // Create UDP PCB
    g_xcp_udp_state.connection = udp_new();
    
    // Bind to specific port
    ip_addr_t bind_addr;
    udp_bind(g_xcp_udp_state.connection, &bind_addr, XCP_UDP_PORT);
    
    // Set receive callback
    udp_recv(g_xcp_udp_state.connection, xcp_udp_recv, NULL);
    
    return HAL_OK;
}

// Send UDP XCP Response
HAL_StatusTypeDef XCP_Udp_Send_Response(uint8_t* data, uint16_t length) 
{
    struct pbuf* response_buf = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
    memcpy(response_buf->payload, data, length);
    udp_sendto(g_xcp_udp_state.connection, response_buf, &g_xcp_udp_state.remote_addr, g_xcp_udp_state.remote_port);
    pbuf_free(response_buf);
    return HAL_OK;
}
