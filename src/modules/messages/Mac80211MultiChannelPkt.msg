//**************************************************************************
// * file:        Mac80211MultiChannelPkt.msg
// *
// * author:      
// * copyright:   
// *              
// *              
// * email:       
// **************************************************************************
// * part of:     framework implementation developed by tkn
// *************************************************************************/

cplusplus {{
#include "MiXiMMacPkt.h"
#include "SimpleAddress.h"
}}


class MacPkt;
class LAddress::L2Type extends void;

//
// Defines all fields of an 802.11 MAC frame
//
packet Mac80211MultiChannelPkt extends MacPkt
{
    int address3;
    int address4;
    int fragmentation; //part of the Frame Control field
    int informationDS; //part of the Frame Control field
    int sequenceControl;
    bool retry;
    simtime_t duration; 	//the expected remaining duration the current transaction 
						//(RTS->CTS->DATA->ACK) between two macs will occupy the channel
						
	LAddress::L2Type schSender;
	LAddress::L2Type schReceiver;
	int selectedSch;
	simtime_t schDuration;
}
