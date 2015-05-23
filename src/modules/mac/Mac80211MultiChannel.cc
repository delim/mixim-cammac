#include "mac/Mac80211MultiChannel.h"

#include "PhyToMacControlInfo.h"
#include "FWMath.h"
#include "phy/Decider80211.h"
#include "DeciderResult80211.h"
#include "BaseConnectionManager.h"
#include "MacToPhyInterface.h"
#include "ChannelSenseRequest_m.h"
#include "BasePhyLayer.h"

Define_Module(Mac80211MultiChannel);

Mac80211MultiChannel::Mac80211MultiChannel()
    : BaseMacLayer()
    , timeout()
    , nav(NULL)
    , sch_nav()
    , reattempt(NULL)
    , contention(NULL)
    , endSifs(NULL)
    , channelQuiet(NULL)
    , chSenseStart()
    , state()
    , defaultBitrate(0)
    , txPower(0)
    , centerFreq(0)
    , bitrate(0)
    , autoBitrate(false)
    , snrThresholds()
    , queueLength(0)
    , nextIsBroadcast(false)
    , fromUpperLayer()
    , longRetryCounter(0)
    , shortRetryCounter(0)
    , remainingBackoff()
    , currentIFS()
    , rtsCtsThreshold(0)
    , delta()
    , neighborhoodCacheSize(0)
    , neighborhoodCacheMaxAge()
    , neighbors()
    , switching(false)
    , fsc(0)
    , SCHchannelInfo()
    , selectedSCH(0)
{}

void Mac80211MultiChannel::initialize(int stage)
{
    BaseMacLayer::initialize(stage);

    if (stage == 0)
    {
        debugEV << "Initializing stage 0\n";

        switching = false;
        fsc = intrand(0x7FFFFFFF);
        if(fsc == 0) fsc = 1;
        debugEV << " fsc: " << fsc << "\n";

        queueLength = hasPar("queueLength") ? par("queueLength").longValue() : 10;

        // timers
        timeout = new cMessage("timeout", TIMEOUT);
        nav = new cMessage("NAV", NAV);

        sch_nav[0] = new cMessage("sch0_NAV", SCH0_NAV);
        sch_nav[1] = new cMessage("sch1_NAV", SCH1_NAV);
        sch_nav[2] = new cMessage("sch2_NAV", SCH2_NAV);
        sch_nav[3] = new cMessage("sch3_NAV", SCH3_NAV);
        sch_nav[4] = new cMessage("sch4_NAV", SCH4_NAV);
        sch_nav[5] = new cMessage("sch5_NAV", SCH5_NAV);
        sch_nav[6] = new cMessage("sch6_NAV", SCH6_NAV);
        sch_nav[7] = new cMessage("sch7_NAV", SCH7_NAV);
        sch_nav[8] = new cMessage("sch8_NAV", SCH8_NAV);
        sch_nav[9] = new cMessage("sch9_NAV", SCH9_NAV);
        sch_nav[10] = new cMessage("sch10_NAV", SCH10_NAV);
        sch_nav[11] = new cMessage("sch11_NAV", SCH11_NAV);
        sch_nav[12] = new cMessage("sch12_NAV", SCH12_NAV);

        reattempt = new cMessage("reattempt", REATTEMPT);

        // service channels table
        for (int i=0; i<NO_OF_CHANNELS; i++) {
            SCHchannelInfo[i].occupied = false;
            SCHchannelInfo[i].sender = LAddress::L2NULL;
            SCHchannelInfo[i].receiver = LAddress::L2NULL;
        }

        contention = new ChannelSenseRequest("contention", MacToPhyInterface::CHANNEL_SENSE_REQUEST);
        contention->setSenseMode(UNTIL_BUSY);
        endSifs = new ChannelSenseRequest("end SIFS", MacToPhyInterface::CHANNEL_SENSE_REQUEST);
        endSifs->setSenseMode(UNTIL_BUSY);
        endSifs->setSenseTimeout(SIFS);

        channelQuiet = new ChannelSenseRequest("channel quiet", MacToPhyInterface::CHANNEL_SENSE_REQUEST);
        channelQuiet->setSenseMode(UNTIL_IDLE);
        channelQuiet->setSenseTimeout(packetDuration(LENGTH_INV, 2E+6));

        state = IDLE;
        longRetryCounter = 0;
        shortRetryCounter = 0;
        rtsCtsThreshold = hasPar("rtsCtsThreshold") ? par("rtsCtsThreshold").longValue() : 1;
        currentIFS = EIFS;

        autoBitrate = hasPar("autoBitrate") ? par("autoBitrate").boolValue() : false;

        txPower = hasPar("txPower") ? par("txPower").doubleValue() : 110.11;

        delta = 1E-9;

        debugEV << "SIFS: " << SIFS << " DIFS: " << DIFS << " EIFS: " << EIFS << endl;
    }
    else if(stage == 1) {
        BaseConnectionManager* cc = getConnectionManager();

        if(cc->hasPar("pMax") && txPower > cc->par("pMax").doubleValue())
            opp_error("TranmitterPower can't be bigger than pMax in ConnectionManager! "
                      "Please adjust your omnetpp.ini file accordingly.");

        int channel = phy->getCurrentRadioChannel();
        if(!(1<=channel && channel<=14)) {
            opp_error("MiximRadio set to invalid channel %d. Please make sure the"
                      " phy modules parameter \"initialRadioChannel\" is set to"
                      " a valid 802.11 channel (1 to 14)!", channel);
        }
        centerFreq = CENTER_FREQUENCIES[channel];

        bool found = false;
        bitrate = hasPar("bitrate") ? par("bitrate").doubleValue() : BITRATES_80211[0];
        for(int i = 0; i < 4; i++) {
            if(bitrate == BITRATES_80211[i]) {
                found = true;
                break;
            }
        }
        if(!found) bitrate = BITRATES_80211[0];
        defaultBitrate = bitrate;

        snrThresholds.push_back(hasPar("snr2Mbit") ? par("snr2Mbit").doubleValue() : 100);
        snrThresholds.push_back(hasPar("snr5Mbit") ? par("snr5Mbit").doubleValue() : 100);
        snrThresholds.push_back(hasPar("snr11Mbit") ? par("snr11Mbit").doubleValue() : 100);
        snrThresholds.push_back(111111111); // sentinel

        neighborhoodCacheSize = hasPar("neighborhoodCacheSize") ? par("neighborhoodCacheSize").longValue() : 0;
        neighborhoodCacheMaxAge = hasPar("neighborhoodCacheMaxAge") ? par("neighborhoodCacheMaxAge").longValue() : 10000;

        debugEV << " MAC Address: " << myMacAddr
           << " rtsCtsThreshold: " << rtsCtsThreshold
           << " bitrate: " << bitrate
           << " channel: " << channel
           << " autoBitrate: " << autoBitrate
           << " 2MBit: " << snrThresholds[0]
           << " 5.5MBit: " <<snrThresholds[1]
           << " 11MBit: " << snrThresholds[2]
           << " neighborhoodCacheSize " << neighborhoodCacheSize
           << " neighborhoodCacheMaxAge " << neighborhoodCacheMaxAge
           << endl;

        for(int i = 0; i < 3; i++) {
            snrThresholds[i] = FWMath::dBm2mW(snrThresholds[i]);
        }

        remainingBackoff = backoff();
        senseChannelWhileIdle(remainingBackoff + currentIFS);
    }
}

void Mac80211MultiChannel::switchChannel(int channel) {
    if (!(1 <= channel && channel <= 14))
    {
        opp_error("Invalid channel %d. Mac tried to switch to a channel not"
                " supported by this protocoll.", channel);
    }
    phy->setCurrentRadioChannel(channel);

    centerFreq = CENTER_FREQUENCIES[channel];
}

int Mac80211MultiChannel::getChannel() const
{
    return phy->getCurrentRadioChannel();
}

void Mac80211MultiChannel::senseChannelWhileIdle(simtime_t_cref duration) {
    if(contention->isScheduled()) {
        error("Cannot start a new channel sense request because already sensing the channel!");
    }

    chSenseStart = simTime();
    contention->setSenseTimeout(duration);

    sendControlDown(contention);
}

/**
 * This implementation does not support fragmentation, so it is tested
 * if the maximum length of the MPDU is exceeded.
 */
void Mac80211MultiChannel::handleUpperMsg(cMessage *msg)
{
    cPacket* pkt = static_cast<cPacket*>(msg);

    debugEV << "Mac80211MultiChannel::handleUpperMsg " << msg->getName() << "\n";
    if (pkt->getBitLength() > 18496){
        error("packet from higher layer (%s)%s is too long for 802.11b, %d bytes (fragmentation is not supported yet)",
              pkt->getClassName(), pkt->getName(), pkt->getByteLength());
    }

    if(fromUpperLayer.size() == queueLength) {
        //TODO: CSMAMacLayer does create a new mac packet and sends it up. Maybe settle on a consistent solution here
        msg->setName("MAC ERROR");
        msg->setKind(PACKET_DROPPED);
        sendControlUp(msg);
        debugEV << "packet " << msg << " received from higher layer but MAC queue is full, signalling error\n";
        return;
    }

    Mac80211MultiChannelPkt *mac = static_cast<Mac80211MultiChannelPkt*>(encapsMsg(pkt));
    debugEV << "packet " << pkt << " received from higher layer, dest=" << mac->getDestAddr() << ", encapsulated\n";

    fromUpperLayer.push_back(mac);
    // If the MAC is in the IDLE state, then start a new contention period
    if (state == IDLE && !endSifs->isScheduled()) {
        beginNewCycle();
    }
    else
    {
        debugEV << "enqueued, will be transmitted later\n";
    }
}

/**
 * Encapsulates the received network-layer packet into a MacPkt and set all needed
 * header fields.
 */
Mac80211MultiChannel::macpkt_ptr_t Mac80211MultiChannel::encapsMsg(cPacket* netw)
{

    Mac80211MultiChannelPkt *pkt = new Mac80211MultiChannelPkt(netw->getName());
    // headerLength, including final CRC-field AND the phy header length!
    pkt->setBitLength(MAC80211_HEADER_LENGTH);
    pkt->setRetry(false);                 // this is not a retry
    pkt->setSequenceControl(fsc++);       // add a unique fsc to it
    if(fsc <= 0) fsc = 1;

    // copy dest address from the Control Info attached to the network
    // mesage by the network layer
    cObject *const cInfo = netw->removeControlInfo();

    debugEV << "CInfo removed, mac addr=" << getUpperDestinationFromControlInfo(cInfo) << endl;
    pkt->setDestAddr(getUpperDestinationFromControlInfo(cInfo));

    //delete the control info
    delete cInfo;

    //set the src address to own mac address (nic module id())
    pkt->setSrcAddr(myMacAddr);

    //encapsulate the network packet
    pkt->encapsulate(netw);
    debugEV <<"pkt encapsulated, length: " << pkt->getBitLength() << "\n";

    return pkt;
}

cPacket *Mac80211MultiChannel::decapsMsg(macpkt_ptr_t frame) {
    cPacket *m = frame->decapsulate();
    setUpControlInfo(m, frame->getSrcAddr());
    debugEV << " message decapsulated " << endl;
    return m;
}

/**
 *  Handle all messages from lower layer. Checks the destination MAC
 *  adress of the packet. Then calls one of the three functions :
 *  handleMsgNotForMe(), handleBroadcastMsg(), or handleMsgForMe().
 *  Called by handleMessage().
 */
void Mac80211MultiChannel::handleLowerMsg(cMessage *msg)
{
    Mac80211MultiChannelPkt *af = static_cast<Mac80211MultiChannelPkt *>(msg);
    int radioState = phy->getRadioState();
    if(radioState == MiximRadio::RX) {
        // end of the reception
        debugEV << " handleLowerMsg frame " << af << " received from " << af->getSrcAddr() << " addressed to " << af->getDestAddr() << "\n";
        addNeighbor(af);
        if (contention->isScheduled()) {
            error("Gaack! I am changing the IFS on an ongoing contention");
        }
        currentIFS = DIFS;
        if(af->getDestAddr() == myMacAddr) {
            handleMsgForMe(af);
        }
        else if(LAddress::isL2Broadcast(af->getDestAddr())) {
            handleBroadcastMsg(af);
        }
        else {
            handleMsgNotForMe(af, af->getDuration());
        }
    }
    else {
        debugEV << " handleLowerMsg frame " << af << " deleted, strange race condition\n";
        delete af;
    }
}

void Mac80211MultiChannel::handleLowerControl(cMessage *msg)
{
    debugEV << simTime() << " handleLowerControl " << msg->getName() << "\n";
    switch(msg->getKind()) {
    case MacToPhyInterface::CHANNEL_SENSE_REQUEST:
        if(msg == contention) {
            handleEndContentionTimer();
        } else if(msg == endSifs) {
            handleEndSifsTimer();   // noch zu betrachten
        } else if(msg == channelQuiet) {
            beginNewCycle();
        } else {
            error("Unknown ChannelSenseRequest returned!");
        }
        break;

    case Decider80211::COLLISION:
    case BaseDecider::PACKET_DROPPED:
    {
        int radioState = phy->getRadioState();
        if(radioState == MiximRadio::RX) {
            if (contention->isScheduled()) {
                error("Gaack! I am changing the IFS on an ongoing contention");
            }
            currentIFS = EIFS;
            handleMsgNotForMe(msg, 0);
        }
        else {
            debugEV << " frame " << msg->getName() << " deleted, strange race condition\n";
            delete msg;
        }
        break;
    }
    case MacToPhyInterface::TX_OVER:
        debugEV << "PHY indicated transmission over" << endl;
        phy->setRadioState(MiximRadio::RX);
        handleEndTransmission();
        delete msg;
        break;

    case MacToPhyInterface::RADIO_SWITCHING_OVER:
        //TODO: handle this correctly (by now we assume switch times are zero)
        delete msg;
        break;

    default:
        ev << "Unhandled control message from physical layer: " << msg << endl;
        delete msg;
        break;
    }
}

/**
 * handle timers
 */
void Mac80211MultiChannel::handleSelfMsg(cMessage * msg)
{
    debugEV << simTime() << " handleSelfMsg " << msg->getName() << "\n";
    switch (msg->getKind())
    {
        // the MAC was waiting for a CTS, a DATA, or an ACK packet but the timer has expired.
    case TIMEOUT:
        handleTimeoutTimer();   // noch zu betrachten..
        break;

        // the MAC was waiting because an other communication had won the channel. This communication is now over
    case NAV:
        handleNavTimer();       // noch zu betrachten...
        break;

    //start DAO
    case REATTEMPT:
        if (state == IDLE && !endSifs->isScheduled())
            beginNewCycle();
        break;

    case SCH0_NAV:
        SCHchannelInfo[0].occupied = false;
        SCHchannelInfo[0].sender = LAddress::L2NULL;
        SCHchannelInfo[0].receiver = LAddress::L2NULL;
        break;

    case SCH1_NAV:
        SCHchannelInfo[1].occupied = false;
        SCHchannelInfo[1].sender = LAddress::L2NULL;
        SCHchannelInfo[1].receiver = LAddress::L2NULL;
        break;

    case SCH2_NAV:
        SCHchannelInfo[2].occupied = false;
        SCHchannelInfo[2].sender = LAddress::L2NULL;
        SCHchannelInfo[2].receiver = LAddress::L2NULL;
        break;

    case SCH3_NAV:
        SCHchannelInfo[3].occupied = false;
        SCHchannelInfo[3].sender = LAddress::L2NULL;
        SCHchannelInfo[3].receiver = LAddress::L2NULL;
        break;

    case SCH4_NAV:
        SCHchannelInfo[4].occupied = false;
        SCHchannelInfo[4].sender = LAddress::L2NULL;
        SCHchannelInfo[4].receiver = LAddress::L2NULL;
        break;

    case SCH5_NAV:
        SCHchannelInfo[5].occupied = false;
        SCHchannelInfo[5].sender = LAddress::L2NULL;
        SCHchannelInfo[5].receiver = LAddress::L2NULL;
        break;

    case SCH6_NAV:
        SCHchannelInfo[6].occupied = false;
        SCHchannelInfo[6].sender = LAddress::L2NULL;
        SCHchannelInfo[6].receiver = LAddress::L2NULL;
        break;

    case SCH7_NAV:
        SCHchannelInfo[7].occupied = false;
        SCHchannelInfo[7].sender = LAddress::L2NULL;
        SCHchannelInfo[7].receiver = LAddress::L2NULL;
        break;

    case SCH8_NAV:
        SCHchannelInfo[8].occupied = false;
        SCHchannelInfo[8].sender = LAddress::L2NULL;
        SCHchannelInfo[8].receiver = LAddress::L2NULL;
        break;

    case SCH9_NAV:
        SCHchannelInfo[9].occupied = false;
        SCHchannelInfo[9].sender = LAddress::L2NULL;
        SCHchannelInfo[9].receiver = LAddress::L2NULL;
        break;

    case SCH10_NAV:
        SCHchannelInfo[10].occupied = false;
        SCHchannelInfo[10].sender = LAddress::L2NULL;
        SCHchannelInfo[10].receiver = LAddress::L2NULL;
        break;

    case SCH11_NAV:
        SCHchannelInfo[11].occupied = false;
        SCHchannelInfo[11].sender = LAddress::L2NULL;
        SCHchannelInfo[11].receiver = LAddress::L2NULL;
        break;

    case SCH12_NAV:
        SCHchannelInfo[12].occupied = false;
        SCHchannelInfo[12].sender = LAddress::L2NULL;
        SCHchannelInfo[12].receiver = LAddress::L2NULL;
        break;

    //end DAO

    default:
        error("unknown timer type");
        break;
    }
}

/**
 *  Handle all ACKs,RTS, CTS, or DATA not for the node. If RTS/CTS
 *  is used the node must stay quiet until the current handshake
 *  between the two communicating nodes is over.  This is done by
 *  scheduling the timer message nav (Network Allocation Vector).
 *  Without RTS/CTS a new contention is started. If an error
 *  occured the node must defer for EIFS.  Called by
 *  handleLowerMsg()
 */
void Mac80211MultiChannel::handleMsgNotForMe(cMessage *af, simtime_t_cref duration)
{
    debugEV << "handle msg not for me " << af->getName() << "\n";

    Mac80211MultiChannelPkt *msg = static_cast<Mac80211MultiChannelPkt *>(af);

    // if the message is a PRA(RTS)/PRB(CTS), validate it
    if (af->getKind() == RTS || af->getKind() == CTS) {
        if (state != QUIET) {

            // if the service channel selected by the PRA/PRB is unavailable
            if (!SCH_free(msg->getSelectedSch())) {

                // if the MAC wait for another frame, it can delete its time out
                // (exchange is aborted)
                if (timeout->isScheduled()) {
                    cancelEvent(timeout);
                    if(state == WFACK) {
                        fromUpperLayer.front()->setRetry(true);
                    }
                    if((state == WFACK) || (state == WFCTS) || (state == WFCFB)) {
                        if(rtsCts(fromUpperLayer.front())) {
                            longRetryCounter++;
                        }
                        else {
                            shortRetryCounter++;
                        }
                    }
                }
                assert(!contention->isScheduled());

                // transmit an INV frame and exit
                sendINVframe(msg);

                delete af;
                return;
            }
        }
    }

    else if (af->getKind() == CFA || af->getKind() == CFB || af->getKind() == OBJ) {
        // update service channel NAVs using handleINVframe()
        handleINVframe(msg);

        if(state == WFCTS) {
            assert(timeout->isScheduled());
            cancelEvent(timeout);
            rtsTransmissionFailed();
        }
    }

    // if the duration of the packet is null, then do nothing (to avoid
    // the unuseful scheduling of a self message)
    if(duration != 0) {
        // the node is already deferring
        if (state == QUIET)
        {
            // the current value of the NAV is not sufficient
            if (nav->getArrivalTime() < simTime() + duration)
            {
                cancelEvent(nav);
                scheduleAt(simTime() + duration, nav);
                debugEV << "NAV timer started for: " << duration << " State QUIET\n";
            }
        }
        // other states
        else
        {
            // if the MAC wait for another frame, it can delete its time out
            // (exchange is aborted)
            if (timeout->isScheduled()) {
                cancelEvent(timeout);
                if(state == WFACK) {
                    fromUpperLayer.front()->setRetry(true);
                }
                if((state == WFACK) || (state == WFCTS) || (state == WFCFB)) {
                    if(rtsCts(fromUpperLayer.front())) {
                        longRetryCounter++;
                    }
                    else {
                        shortRetryCounter++;
                    }
                }
            }
            // the node must defer for the time of the transmission
            scheduleAt(simTime() + duration, nav);
            debugEV << "NAV timer started, not QUIET: " << duration << endl;

            assert(!contention->isScheduled());
            //suspendContention();

            setState(QUIET);
        }
    }

    if((af->getKind() == BaseDecider::PACKET_DROPPED) || (af->getKind() == Decider80211::COLLISION)) {

        assert(!contention->isScheduled());
        //suspendContention();
        //if (contention->isScheduled()) {
        //    error("Gaack! I am changing the IFS on an ongoing contention");
        //}

        //handle broken cts and ACK frames
        if(state == WFCTS || state == WFCFB) {
            assert(timeout->isScheduled());
            cancelEvent(timeout);
            rtsTransmissionFailed();
        }
        else if(state == WFACK) {
            assert(timeout->isScheduled());
            cancelEvent(timeout);
            dataTransmissionFailed();
        }

        currentIFS = EIFS;
    }

    beginNewCycle();
    delete af;
}

/**
 *  Handle a packet for the node. The result of this reception is
 *  a function of the type of the received message (RTS,CTS,DATA,
 *  or ACK), and of the current state of the MAC (WFDATA, CONTEND,
 *  IDLE, WFCTS, or WFACK). Called by handleLowerMsg()
 */
void Mac80211MultiChannel::handleMsgForMe(Mac80211MultiChannelPkt *af)
{
    debugEV << "handle msg for me " << af->getName() << " in " <<  stateName(state) << "\n";

    switch (state)
    {
    case IDLE:     // waiting for the end of the contention period
    case CONTEND:  // or waiting for RTS

        // RTS, CFA or DATA accepted
        if (af->getKind() == RTS) {
            assert(!contention->isScheduled());
            //suspendContention();

            handleRTSframe(af);
        }
        else if (af->getKind() == CFA) {
            assert(!contention->isScheduled());
            //suspendContention();

            // regenerate backoff
            remainingBackoff = backoff();

            handleCFAframe(af);
        }
        else if (af->getKind() == DATA) {
            assert(!contention->isScheduled());
            //suspendContention();

            handleDATAframe(af);
        }
        else {
            error("in handleMsgForMe() IDLE/CONTEND, strange message %s", af->getName());
        }
        break;

    case WFDATA:  // waiting for DATA
        if (af->getKind() == DATA) {
            handleDATAframe(af);
        }
        else {
            EV << "unexpected message -- probably a collision of RTSs\n";
            delete af;
        }
        break;

    case WFACK:  // waiting for ACK
        if (af->getKind() == ACK) {
            handleACKframe(af);
        }
        else {
            error("in handleMsgForMe() WFACK, strange message %s", af->getName());
        }
        delete af;
        break;

    case WFCTS:  // The MAC is waiting for CTS
        if (af->getKind() == CTS) {
            handleCTSframe(af);
        }
        else {
            EV << "unexpected msg -- deleted \n";
            delete af;
        }
        break;

    case WFCFB:  // The MAC is waiting for CFB
        if (af->getKind() == CFB) {
            handleCFBframe(af);
        }
        else {
            EV << "unexpected msg -- deleted \n";
            delete af;
        }
        break;

    case QUIET: // the node is currently deferring.

        // cannot handle any packet with its MAC adress
        delete af;
        break;

    case BUSY: // currently transmitting an ACK or a BROADCAST packet
        if(switching) {
            EV << "message received during radio state switchover\n";
            delete af;
        }
        else {
            error("logic error: node is currently transmitting, can not receive "
                  "(does the physical layer do its job correctly?)");
        }
        break;
    default:
        error("unknown state %d", state);
        break;
    }
}

/**
 *  Handle aframe wich is expected to be an RTS. Called by
 *  HandleMsgForMe()
 */
void Mac80211MultiChannel::handleRTSframe(Mac80211MultiChannelPkt * af)
{
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleRTSframe when SIFS scheduled");
    // wait a short interframe space
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

/**
 *  Handle a frame which expected to be a DATA frame. Called by
 *  HandleMsgForMe()
 */
void Mac80211MultiChannel::handleDATAframe(Mac80211MultiChannelPkt * af)
{
    NeighborList::iterator it;
    if (rtsCts(af)) cancelEvent(timeout);  // cancel time-out event
    it = findNeighbor(af->getSrcAddr());
    if(it == neighbors.end()) error("Mac80211MultiChannel::handleDATAframe: neighbor not registered");
    if(af->getRetry() && (it->fsc == af->getSequenceControl())) {
        debugEV << "Mac80211MultiChannel::handleDATAframe suppressed duplicate message " << af
           << " fsc: " << it->fsc << "\n";
    }
    else {
        it->fsc = af->getSequenceControl();
        // pass the packet to the upper layer
        sendUp(decapsMsg(af));
    }
    // wait a short interframe space
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleDATAframe when SIFS scheduled");
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

/**
 *  Handle ACK and delete corresponding packet from queue
 */
void Mac80211MultiChannel::handleACKframe(Mac80211MultiChannelPkt * /*af*/)
{
    // cancel time-out event
    cancelEvent(timeout);

    // the transmission is acknowledged : initialize long_retry_counter
    longRetryCounter = 0;
    shortRetryCounter = 0;
    // post transmit backoff
    remainingBackoff = backoff();
    // removes the acknowledged packet from the queue
    Mac80211MultiChannelPkt *temp = fromUpperLayer.front();
    fromUpperLayer.pop_front();
    delete temp;

    // if thre's a packet to send and if the channel is free then start a new contention period
    beginNewCycle();
}

/**
 *  Handle a CTS frame. Called by HandleMsgForMe(Mac80211MultiChannelPkt* af)
 */
void Mac80211MultiChannel::handleCTSframe(Mac80211MultiChannelPkt * af)
{
    // cancel time-out event
    cancelEvent(timeout);
    //shortRetryCounter = 0;
    // wait a short interframe space
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleCTSframe when SIFS scheduled");
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

/**
 *  Handle a broadcast packet. This packet is simply passed to the
 *  upper layer. No acknowledgement is needed.  Called by
 *  handleLowerMsg(Mac80211MultiChannelPkt *af)
 */
void Mac80211MultiChannel::handleBroadcastMsg(Mac80211MultiChannelPkt *af)
{
    debugEV << "handle broadcast\n";
    if((state == BUSY) && (!switching)) {
        error("logic error: node is currently transmitting, can not receive "
              "(does the physical layer do its job correctly?)");
    }
    if (af->getKind() == BROADCAST) {
        sendUp(decapsMsg(af));
    }
    else if (af->getKind() == NCF) {
        handleNCFframe(af);
    }
    delete af;
    if (state == CONTEND) {
        assert(!contention->isScheduled());
        //suspendContention();

        beginNewCycle();
    }
}

void Mac80211MultiChannel::handleCFAframe(Mac80211MultiChannelPkt * af)
{
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleCFAframe when SIFS scheduled");
    // wait a short interframe space
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

void Mac80211MultiChannel::handleCFBframe(Mac80211MultiChannelPkt * af)
{
    // cancel time-out event
    cancelEvent(timeout);
    shortRetryCounter = 0;
    // wait a short interframe space
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleCFBframe when SIFS scheduled");
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);

    // switch to service channel
    switchChannel(af->getSelectedSch() + 2);
}

void Mac80211MultiChannel::handleNCFframe(Mac80211MultiChannelPkt * af)
{
    // update SCHchannelInfo
    if (af->getSchSender() == SCHchannelInfo[af->getSelectedSch()].sender) {
        SCHchannelInfo[af->getSelectedSch()].occupied = false;
        cancelEvent(sch_nav[af->getSelectedSch()]);
        SCHchannelInfo[af->getSelectedSch()].sender = LAddress::L2NULL;
        SCHchannelInfo[af->getSelectedSch()].receiver = LAddress::L2NULL;
    }

    debugEV << "learned that reservation for service channel " << af->getSelectedSch() << " is canceled\n";
}

void Mac80211MultiChannel::handleINVframe(Mac80211MultiChannelPkt * af)
{
    debugEV << "Mac80211MultiChannel::handleOBJframe learned that service channel " << af->getSelectedSch() + 1
            << " is occupied by " << af->getSchSender() << " and " << af->getSchReceiver() << "." << endl;
    // update SCHchannelInfo
    if (overideSCHNAV(af->getSelectedSch(), af->getSchDuration())) {
        SCHchannelInfo[af->getSelectedSch()].occupied = true;
        SCHchannelInfo[af->getSelectedSch()].sender = af->getSchSender();
        SCHchannelInfo[af->getSelectedSch()].receiver = af->getSchReceiver();
    }
    else {
        debugEV << "But service channel " << af->getSelectedSch() + 1 << " is already occupied by "
                << SCHchannelInfo[af->getSelectedSch()].sender << " and " << SCHchannelInfo[af->getSelectedSch()].receiver
                << "." << endl;
    }
}

/**
 *  The node has won the contention, and is now allowed to send an
 *  RTS/DATA or Broadcast packet. The backoff value is deleted and
 *  will be newly computed in the next contention period
 */
void Mac80211MultiChannel::handleEndContentionTimer()
{
    if(!contention->getResult().isIdle()) {
        suspendContention();
        return;
    }

    if(state == IDLE) {
        remainingBackoff = 0;
        // bug: ID: 3460932
        /*
        In Mac80211MultiChannel, currentIFS is set to DIFS upon successful reception and EIFS
        upon reception of a corrupted frame. This is correct. However, currentIFS
        remains unchanged while performing post-backoff. As a result, the EIFS related
        to reception of a corrupted frame (and subsequent post-backoff) is used again in
        the minimum duration of idle time before a transmission is performed (i.e. the
        mandatory IFS during which a station decides whether to transmit directly or go
        do backoff). This results in incorrect behaviour, as this is a new opportunity.

        A fix is simple: in handleEndContentionTimer(), add "currentIFS = DIFS;" inside the
        state==IDLE if-clause. This is where post-backoff completes and not only remainingBackoff
        but also currentIFS should be reset.

        See also:
        http://www.freeminded.org/?p=811
        */
        currentIFS       = DIFS;
    }
    else if(state == CONTEND) {
        // the node has won the channel, the backoff window is deleted and
        // will be new calculated in the next contention period
        remainingBackoff = 0;
        // unicast packet
        phy->setRadioState(MiximRadio::TX);
        if (!nextIsBroadcast)
        {
            if(rtsCts(fromUpperLayer.front())) {
                // send a RTS
                sendRTSframe();
            }
            else {
                sendDATAframe(0);
            }
        }// broadcast packet
        else {
            sendBROADCASTframe();
            // removes the packet from the queue without waiting for an acknowledgement
            Mac80211MultiChannelPkt *temp = fromUpperLayer.front();
            fromUpperLayer.pop_front();
            delete(temp);
        }
    }
    else {
        error("logic error: expiration of the contention timer outside of CONTEND/IDLE state, should not happen");
    }
}

/**
 *  Handle the NAV timer (end of a defering period). Called by
 *  HandleTimer(cMessage* msg)
 */
void Mac80211MultiChannel::handleNavTimer()
{
    if (state != QUIET)
        error("logic error: expiration of the NAV timer outside of the state QUIET, should not happen");

    // if there's a packet to send and if the channel is free, then start a new contention period

    // lmf - Potential race condition: If nav expires at same time
    // medium becomes IDLE (usual case), but the navTimeout comes
    // before the mediumIndication, then the medium is still BUSY when
    // beginNewCycle() is called, so the backoff doesn't resume.
    // Usually it doesn't matter, since there is also an arriving
    // frame and so beginNewCycle() will be called again by the
    // handleMsgXXX.  But if there is no frame (mobility, exposed
    // terminal, etc) there's a potential problem.

    beginNewCycle();
}

void Mac80211MultiChannel::dataTransmissionFailed()
{
    bool rtscts = rtsCts(fromUpperLayer.front());
    if(rtscts){
        longRetryCounter++;
    }else{
        shortRetryCounter++;
    }
    fromUpperLayer.front()->setRetry(true);
    remainingBackoff = backoff(rtscts);
}

void Mac80211MultiChannel::rtsTransmissionFailed()
{
    longRetryCounter++;
    remainingBackoff = backoff();
}

/**
 *  Handle the time out timer. Called by handleTimer(cMessage* msg)
 */
void Mac80211MultiChannel::handleTimeoutTimer()
{
    debugEV << simTime() << " handleTimeoutTimer " << stateName(state) << "\n";
    if(state == WFCTS) {
        rtsTransmissionFailed();
    }
    else if(state == WFCFB) {
        rtsTransmissionFailed();
        phy->setRadioState(MiximRadio::TX);
        sendNCFframe();
        // handleEndTransmission() will begin a new cycle after the NCF is sent
        return;
    }
    else if(state == WFACK) {
        dataTransmissionFailed();
    }
    else if(state == WFDATA) {
        remainingBackoff = backoff();
    }

    // if there's a packet to send and if the channel is free then
    // start a new contention period
    if (state != QUIET)
        beginNewCycle();
}


/**
 *  Handle the end sifs timer. Then sends a CTS, a DATA, or an ACK
 *  frame
 */
void Mac80211MultiChannel::handleEndSifsTimer()
{
    if(!endSifs->getResult().isIdle()){
        // delete the previously received frame
        delete static_cast<Mac80211MultiChannelPkt *>(endSifs->getContextPointer());

        // state in now IDLE or CONTEND
        if (fromUpperLayer.empty())
            setState(IDLE);
        else
            setState(CONTEND);

        return;
    }

    Mac80211MultiChannelPkt *frame = static_cast<Mac80211MultiChannelPkt *>(endSifs->getContextPointer());
    phy->setRadioState(MiximRadio::TX);
    switch (frame->getKind())
    {
    case RTS:
        if (SCH_free(frame->getSelectedSch())) {
            sendCTSframe(frame);
        }
        else {
            sendINVframe(frame);
        }
        break;
    case CTS:
        sendCFAframe(frame);
        break;
    case CFA:
        sendCFBframe(frame);
        break;
    case CFB:
        sendDATAframe(frame);
        break;
    case DATA:
        sendACKframe(frame);
        break;
    default:
        error("logic error: end sifs timer when previously received packet is not RTS/CTS/DATA");
        break;
    }

    // don't need previous frame any more
    delete frame;
}

/**
 *  Handle the end of transmission timer (end of the transmission
 *  of an ACK or a broadcast packet). Called by
 *  HandleTimer(cMessage* msg)
 */
void Mac80211MultiChannel::handleEndTransmission()
{
    debugEV << "transmission of packet is over\n";
    if(state == BUSY) {
        if(nextIsBroadcast) {
            shortRetryCounter = 0;
            longRetryCounter = 0;
            remainingBackoff = backoff();
        }
        beginNewCycle();
    }
    else if(state == WFCFA) {
        beginNewCycle();
    }
    else if(state == WFDATA) {
        // switch to service channel
        switchChannel(selectedSCH + 2);

        // service channel should not be busy
        // if there is traffic, cancel the communication
        /*ChannelState channel = phy->getChannelState();
        if (!channel.isIdle()) {
            cancelEvent(timeout);
            beginNewCycle();
        }*/
    }
}

/**
 *  Send a DATA frame. Called by HandleEndSifsTimer() or
 *  handleEndContentionTimer()
 */
void Mac80211MultiChannel::sendDATAframe(Mac80211MultiChannelPkt *af)
{
    Mac80211MultiChannelPkt *frame = static_cast<Mac80211MultiChannelPkt *>(fromUpperLayer.front()->dup());
    double br;

    if(af) {
        br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();

        delete af->removeControlInfo();
    }
    else {
       br  = retrieveBitrate(frame->getDestAddr());

       if(shortRetryCounter) frame->setRetry(true);
    }
    simtime_t duration = packetDuration(frame->getBitLength(), br);
    setDownControlInfo(frame, createSignal(simTime(), duration, txPower, br));
    // build a copy of the frame in front of the queue'
    frame->setSrcAddr(myMacAddr);
    frame->setKind(DATA);
    frame->setDuration(SIFS + packetDuration(LENGTH_CAMACK, br));

    // schedule time out
    scheduleAt(simTime() + timeOut(DATA, br), timeout);
    debugEV << "sending DATA  to " << frame->getDestAddr() << " with bitrate " << br << endl;
    // send DATA frame
    sendDown(frame);

    // update state and display
    setState(WFACK);
}

/**
 *  Send an ACK frame.Called by HandleEndSifsTimer()
 */
void Mac80211MultiChannel::sendACKframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-ack");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_CAMACK, br), txPower, br));
    frame->setKind(ACK);
    frame->setBitLength((int)LENGTH_CAMACK);

    // the dest address must be the src adress of the RTS or the DATA
    // packet received. The src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());
    frame->setDuration(0);

    sendDown(frame);
    debugEV << "sent ACK frame!\n";

    // update state and display
    setState(BUSY);
}

/**
 *  Send a RTS frame.Called by handleContentionTimer()
 */
void Mac80211MultiChannel::sendRTSframe()
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-rts");

    const Mac80211MultiChannelPkt* frameToSend = fromUpperLayer.front();
    double br = retrieveBitrate(frameToSend->getDestAddr());

    setDownControlInfo(frame, createSignal(simTime(),packetDuration(LENGTH_PRA, br), txPower, br));

    frame->setKind(RTS);
    frame->setBitLength((int)LENGTH_PRA);

    // the src adress and dest address are copied in the frame in the queue (frame to be sent)
    frame->setSrcAddr(frameToSend->getSrcAddr());
    frame->setDestAddr(frameToSend->getDestAddr());

    /*double packetSize = frameToSend->getBitLength();
    frame->setDuration(5 * SIFS + packetDuration(LENGTH_PRB, br) +
                       packetDuration(LENGTH_CFA, br) +
                       packetDuration(LENGTH_CFB, br) +
                       packetDuration(packetSize, br) +
                       packetDuration(LENGTH_CAMACK, br));*/
    frame->setDuration(4 * SIFS + packetDuration(LENGTH_PRB, br) +
                       packetDuration(LENGTH_CFA, br) +
                       packetDuration(LENGTH_CFB, br) +
                       packetDuration(LENGTH_NCF, br));
    frame->setSelectedSch(selectedSCH);

    debugEV << " Mac80211MultiChannel::sendRTSframe duration: " <<  packetDuration(LENGTH_PRA, br) << " br: " << br << "\n";

    // schedule time-out
    scheduleAt(simTime() + timeOut(RTS, br), timeout);

    // send RTS frame
    sendDown(frame);

    // update state and display
    setState(WFCTS);
}

/**
 *  Send a CTS frame.Called by HandleEndSifsTimer()
 */
void Mac80211MultiChannel::sendCTSframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-cts");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_PRB, br), txPower, br));

    frame->setKind(CTS);
    frame->setBitLength((int)LENGTH_PRB);

    // the dest adress must be the src adress of the RTS received. The
    // src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());

    frame->setDuration(af->getDuration() - SIFS - packetDuration(LENGTH_PRB, br));

    frame->setSelectedSch(af->getSelectedSch());

    //scheduleAt(simTime() + af->getDuration() - packetDuration(LENGTH_CAMACK, br) - 2 * SIFS + delta, timeout);
    debugEV << " Mac80211MultiChannel::sendCTSframe duration: " <<  packetDuration(LENGTH_PRB, br) << " br: " << br << "\n";
    // send CTS frame
    sendDown(frame);

    // update state and display
    setState(WFCFA);
}

/**
 *  Send a BROADCAST frame.Called by handleContentionTimer()
 */
void Mac80211MultiChannel::sendBROADCASTframe()
{
    // send a copy of the frame in front of the queue
    Mac80211MultiChannelPkt *frame = static_cast<Mac80211MultiChannelPkt *>(fromUpperLayer.front()->dup());

    double br = retrieveBitrate(frame->getDestAddr());

    simtime_t duration = packetDuration(frame->getBitLength(), br);
    setDownControlInfo(frame, createSignal(simTime(), duration, txPower, br));

    frame->setKind(BROADCAST);

    sendDown(frame);
    // update state and display
    setState(BUSY);
}

void Mac80211MultiChannel::sendCFAframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-cfa");
    const Mac80211MultiChannelPkt* frameToSend = fromUpperLayer.front();

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_CFA, br), txPower, br));

    frame->setKind(CFA);
    frame->setBitLength((int)LENGTH_CFA);

    // the dest adress must be the src adress of the RTS received. The
    // src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());

    simtime_t duration = af->getDuration() - SIFS - packetDuration(LENGTH_CFA, br);
    frame->setDuration(duration);

    // information for other nodes
    frame->setSelectedSch(af->getSelectedSch());
    double packetSize = frameToSend->getBitLength();
    frame->setSchDuration(duration + SIFS +
                          packetDuration(packetSize, br) +
                          packetDuration(LENGTH_ACK, br) -
                          packetDuration(LENGTH_NCF, br));
    frame->setSchSender(myMacAddr);
    frame->setSchReceiver(af->getSrcAddr());

    //scheduleAt(simTime() + af->getDuration() - packetDuration(LENGTH_CAMACK, br) - 2 * SIFS + delta, timeout);
    debugEV << " Mac80211MultiChannel::sendCFAframe duration: " <<  packetDuration(LENGTH_CFA, br) << " br: " << br << "\n";

    scheduleAt(simTime() + timeOut(CFA, br), timeout);

    // send CTS frame
    sendDown(frame);

    // update state and display
    setState(WFCFB);
}

void Mac80211MultiChannel::sendCFBframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-cfb");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_CFB, br), txPower, br));

    frame->setKind(CFB);
    frame->setBitLength((int)LENGTH_CFB);

    // the dest adress must be the src adress of the RTS received. The
    // src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());

    frame->setDuration(af->getDuration() - SIFS - packetDuration(LENGTH_CFB, br));

    // information for other nodes
    frame->setSelectedSch(af->getSelectedSch());
    frame->setSchDuration(af->getSchDuration() - SIFS - packetDuration(LENGTH_CFB, br));
    frame->setSchSender(af->getSrcAddr());
    frame->setSchReceiver(myMacAddr);

    scheduleAt(simTime() + af->getSchDuration() - packetDuration(LENGTH_CAMACK, br) - 2 * SIFS + delta, timeout);
    debugEV << " Mac80211MultiChannel::sendCFBframe duration: " <<  packetDuration(LENGTH_CFB, br) << " br: " << br << "\n";

    // send CFB frame
    sendDown(frame);

    // select service channel to switch
    selectedSCH = af->getSelectedSch();

    // update state and display
    setState(WFDATA);
}

void Mac80211MultiChannel::sendNCFframe()
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-ncf");

    const Mac80211MultiChannelPkt* frameToSend = fromUpperLayer.front();
    double br = retrieveBitrate(frameToSend->getDestAddr());

    setDownControlInfo(frame, createSignal(simTime(),packetDuration(LENGTH_NCF, br), txPower, br));

    frame->setKind(NCF);
    frame->setBitLength((int)LENGTH_NCF);

    // the src adress and dest address are copied in the frame in the queue (frame to be sent)
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(LAddress::L2BROADCAST);

    // information for other nodes
    frame->setSelectedSch(selectedSCH);
    frame->setSchSender(myMacAddr);
    frame->setSchReceiver(frameToSend->getDestAddr());

    sendDown(frame);
    // update state and display
    setState(BUSY);
}

void Mac80211MultiChannel::sendINVframe(Mac80211MultiChannelPkt* af) {

    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-inv");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(),packetDuration(LENGTH_INV, br), txPower, br));

    frame->setKind(OBJ);
    frame->setBitLength((int)LENGTH_INV);

    frame->setSrcAddr(myMacAddr);

    frame->setDuration(0);

    // information for other nodes
    int sch = af->getSelectedSch();

    frame->setSelectedSch(sch);
    frame->setSchDuration(sch_nav[sch]->getArrivalTime() -
                          simTime() - packetDuration(LENGTH_INV, br));

    frame->setSchSender(SCHchannelInfo[sch].sender);
    frame->setSchReceiver(SCHchannelInfo[sch].receiver);

    debugEV << " Mac80211MultiChannel::sendOBJframe duration: " <<  packetDuration(LENGTH_INV, br) << " br: " << br << "\n";
    debugEV << "  Declaring that " <<  SCHchannelInfo[sch].sender << " is communicating with " << SCHchannelInfo[sch].receiver
            << " on service channel: " << sch + 1 << endl;

    // change radio state to transmit
    phy->setRadioState(MiximRadio::TX);

    // send OBJ frame
    sendDown(frame);

    // update state and display
    setState(BUSY);
}

/**
 *  Start a new contention period if the channel is free and if
 *  there's a packet to send.  Called at the end of a deferring
 *  period, a busy period, or after a failure. Called by the
 *  HandleMsgForMe(), HandleTimer() HandleUpperMsg(), and, without
 *  RTS/CTS, by handleMsgNotForMe().
 */
void Mac80211MultiChannel::beginNewCycle()
{
    // before trying to send one more time a packet, test if the
    // maximum retry limit is reached. If it is the case, then
    // delete the packet and send the next packet.
    testMaxAttempts();

    // switch back to control channel
    if (getChannel() != 1) {
        switchChannel(1);
    }

    if (nav->isScheduled()) {
        debugEV << "cannot beginNewCycle until NAV expires at t " << nav->getArrivalTime() << endl;
        return;
    }

    /*
    if(timeout->isScheduled()) {
        cancelEvent(timeout);
    }
    */

    if (!fromUpperLayer.empty()) {

        // look if the next packet is unicast or broadcast
        nextIsBroadcast = LAddress::isL2Broadcast(fromUpperLayer.front()->getDestAddr());

        setState(CONTEND);
        if(!contention->isScheduled()) {

            /* Perform a series of checks */

            // if there are no immediate available service channels
            if (!availableSCH()) {
                // schedule NAV and remain QUIET until a service channel is free
                // NAV will prompt a new beginNewCycle();
                // (no point accepting a handshake but can still send out INV)

                //scheduleAt(getEarliestArrivalSCHNAV(), nav);
                //setState(QUIET);
                cancelEvent(reattempt);
                scheduleAt(getEarliestArrivalSCHNAV(), reattempt);
                setState(IDLE);
                return;
            }

            // or if the recipient is currently in a service channel (unavailable)
            // note that the information may be overwritten even though the recipient is still in the service channel
            int recepientSCH = isInSCH(fromUpperLayer.front()->getDestAddr());
            if (recepientSCH != -1) {
                // schedule reattempt and remain IDLE until the recipient returns
                // (while waiting, node can answer handshakes unless NAV is scheduled)
                cancelEvent(reattempt);
                scheduleAt(sch_nav[recepientSCH]->getArrivalTime(), reattempt);
                setState(IDLE);
                return;
            }
            /* End of checks */

            // select a service channel
            if (longRetryCounter == 0) {
                selectAvailableSCH();
            }
            // else if node was cockblocked then cycle to the next available one
            else {
                cycleNextAvailableSCH(selectedSCH);
            }

            ChannelState channel = phy->getChannelState();
            debugEV << simTime() << " do contention: medium = " << channel.info() << ", backoff = "
               <<  remainingBackoff << endl;

            if(channel.isIdle()) {
                senseChannelWhileIdle(currentIFS + remainingBackoff);
                //scheduleAt(simTime() + currentIFS + remainingBackoff, contention);
            }
            else {
                /*
                Mac80211 in MiXiM uses the same mechanism for backoff and post-backoff, a senseChannelWhileIdle which
                schedules a timer for a duration of IFS + remainingBackoff (i.e. The inter-frame spacing and the present
                state of the backoff counter).

                If Host A were doing post-backoff when the frame from Host B arrived, the remainingBackoff would have
                been > 0 and backoff would have resumed after the frame from Host B finishes.

                However, Host A has already completed its post-backoff (remainingBackoff was 0) so it essentially was
                IDLE when the beacon was generated (actually, it was receiving Host B's frame). So what happens now is
                that all nodes which have an arrival during Host B's frame AND have completed their post-backoff, will
                wait one IFS and then transmit, resulting in synchronised collisions one IFS after a transmission (or
                an EIFS after a collision).

                The correct behaviour here is for Host A's MAC to note that post-backoff has completed (remainingBackoff
                has reached 0) and the medium is busy, so the MAC must draw a new backoff from the contention window.

                http://www.freeminded.org/?p=801
                */
                //channel is busy
                if(remainingBackoff==0) {
                    remainingBackoff = backoff();
                }

                //sendControlDown(channelQuiet);
            }
        }
    }
    else {
        // post-xmit backoff (minor nit: if random backoff=0, we punt)
        if(remainingBackoff > 0 && !contention->isScheduled()) {
            ChannelState channel = phy->getChannelState();
            debugEV << simTime() << " do contention: medium = " << channel.info() << ", backoff = "
               <<  remainingBackoff << endl;

            if(channel.isIdle()) {
                senseChannelWhileIdle(currentIFS + remainingBackoff);
                //scheduleAt(simTime() + currentIFS + remainingBackoff, contention);
            }
        }
        setState(IDLE);
    }
}

/**
 * Compute the backoff value.
 */
simtime_t Mac80211MultiChannel::backoff(bool rtscts) {
    unsigned rc = (rtscts) ?  longRetryCounter : shortRetryCounter;
    unsigned cw = ((CW_MIN + 1) << rc) - 1;
    if(cw > CW_MAX) cw = CW_MAX;

    simtime_t value = ((double) intrand(cw + 1)) * ST;
    debugEV << simTime() << " random backoff = " << value << endl;

    return value;
}

/**
 *  Test if the maximal retry limit is reached, and delete the
 *  frame to send in this case.
 */
void Mac80211MultiChannel::testMaxAttempts()
{
    if ((longRetryCounter >= LONG_RETRY_LIMIT) || (shortRetryCounter >= SHORT_RETRY_LIMIT)) {
        debugEV << "retry limit reached src: "<< shortRetryCounter << " lrc: " << longRetryCounter << endl;
        // initialize counter
        longRetryCounter = 0;
        shortRetryCounter = 0;
        // delete the frame to transmit
        Mac80211MultiChannelPkt *temp = fromUpperLayer.front();
        fromUpperLayer.pop_front();
        temp->setName("MAC ERROR");
        temp->setKind(PACKET_DROPPED);
        sendControlUp(temp);
    }
}

Signal* Mac80211MultiChannel::createSignal( simtime_t_cref start, simtime_t_cref length,
                                double power, double bitrate)
{
    simtime_t end = start + length;
    //create signal with start at current simtime and passed length
    Signal* s = new Signal(start, length);

    //create and set tx power mapping
    ConstMapping* txPowerMapping
            = createSingleFrequencyMapping( start, end,
                                            centerFreq, 11.0e6,
                                            power);
    s->setTransmissionPower(txPowerMapping);

    //create and set bitrate mapping

    //create mapping over time
    Mapping* bitrateMapping
            = MappingUtils::createMapping(DimensionSet::timeDomain,
                                          Mapping::STEPS);

    Argument pos(start);
    bitrateMapping->setValue(pos, BITRATE_HEADER);

    pos.setTime(PHY_HEADER_LENGTH / BITRATE_HEADER);
    bitrateMapping->setValue(pos, bitrate);

    s->setBitrate(bitrateMapping);

    return s;
}

/**
 *  Return a time-out value for a type of frame. Called by
 *  SendRTSframe, sendCTSframe, etc.
 */
simtime_t Mac80211MultiChannel::timeOut(Mac80211MultiChannelMessageKinds type, double br)
{
    simtime_t time_out = 0;

    switch (type)
    {
    case RTS:
        time_out = SIFS + packetDuration(LENGTH_PRA, br) + ST + packetDuration(LENGTH_PRB, br) + delta;
        debugEV << " Mac80211MultiChannel::timeOut RTS " << time_out << "\n";
        break;
    case CFA:
        time_out = SIFS + packetDuration(LENGTH_CFA, br) + ST + packetDuration(LENGTH_CFB, br) + delta;
        debugEV << " Mac80211MultiChannel::timeOut CFA " << time_out << "\n";
        break;
    case CFB:
        time_out = SIFS + packetDuration(LENGTH_CFB, br) + ST + packetDuration(LENGTH_NCF, br) + delta;
        debugEV << " Mac80211MultiChannel::timeOut CFB " << time_out << "\n";
        break;
    case DATA:
        time_out = SIFS + packetDuration(fromUpperLayer.front()->getBitLength(), br) + ST + packetDuration(LENGTH_CAMACK, br) + delta;
        debugEV << " Mac80211MultiChannel::timeOut DATA " << time_out << "\n";
        break;
    default:
        EV << "Unused frame type was given when calling timeOut(), this should not happen!\n";
        break;
    }
    return time_out;
}

/**
 * Computes the duration of the transmission of a frame over the
 * physical channel. 'bits' should be the total length of the
 * mac packet in bits excluding the phy header length.
 */
simtime_t Mac80211MultiChannel::packetDuration(double bits, double br)
{
    return bits / br + phyHeaderLength / BITRATE_HEADER;
}

const char *Mac80211MultiChannel::stateName(State state)
{
#define CASE(x) case x: s=#x; break
    const char *s = "???";
    switch (state)
    {
        CASE(WFDATA);
        CASE(QUIET);
        CASE(IDLE);
        CASE(CONTEND);
        CASE(WFCTS);
        CASE(WFACK);
        CASE(BUSY);
        CASE(WFCFB);
        CASE(WFCFA);
    }
    return s;
#undef CASE
}

void Mac80211MultiChannel::setState(State newState)
{
    if (state==newState)
        debugEV << "staying in state " << stateName(state) << "\n";
    else
        debugEV << "state " << stateName(state) << " --> " << stateName(newState) << "\n";
    state = newState;
}

void Mac80211MultiChannel::suspendContention()  {
    assert(!contention->isScheduled());
    // if there's a contention period

    //if(requestReturned || chSenseRequest->isScheduled()) {
        // update the backoff window in order to give higher priority in
        // the next battle

        simtime_t quietTime = simTime() - chSenseStart;

        debugEV << simTime() << " suspend contention: "
           << "began " << chSenseStart
           << ", ends " << chSenseStart + contention->getSenseTimeout()
           << ", ifs " << currentIFS
           << ", quiet time " << quietTime
           << endl;

        if(quietTime < currentIFS) {
            debugEV << "suspended during D/EIFS (no backoff)" << endl;
        }
        else {
            double remainingSlots;
            remainingSlots = SIMTIME_DBL(contention->getSenseTimeout() - quietTime)/ST;

            // Distinguish between (if) case where contention is
            // suspended after an integer number of slots and we
            // _round_ to integer to avoid fp error, and (else) case
            // where contention is suspended mid-slot (e.g. hidden
            // terminal xmits) and we _ceil_ to repeat the partial
            // slot.  Arbitrary value 0.0001 * ST is used to
            // distinguish the two cases, which may a problem if clock
            // skew between nic's is ever implemented.

            if (fabs(ceil(remainingSlots) - remainingSlots) < 0.0001 * ST ||
                fabs(floor(remainingSlots) - remainingSlots) < 0.0001 * ST) {
                remainingBackoff = floor(remainingSlots + 0.5) * ST;
            }
            else {
                remainingBackoff = ceil(remainingSlots) * ST;
            }

            debugEV << "backoff was " << ((contention->getSenseTimeout() - currentIFS))/ST
               << " slots, now " << remainingSlots << " slots remain" << endl;
        }

        debugEV << "suspended backoff timer, remaining backoff time: "
           << remainingBackoff << endl;

    //}
}

double Mac80211MultiChannel::retrieveBitrate(const LAddress::L2Type& destAddress) {
    double bitrate = defaultBitrate;
    NeighborList::iterator it;
    if(autoBitrate && !LAddress::isL2Broadcast(destAddress) &&
       (longRetryCounter == 0) && (shortRetryCounter == 0)) {
        it = findNeighbor(destAddress);
        if((it != neighbors.end()) && (it->age > (simTime() - neighborhoodCacheMaxAge))) {
            bitrate = it->bitrate;
        }
    }
    return bitrate;
}

void Mac80211MultiChannel::addNeighbor(Mac80211MultiChannelPkt *af) {
    const LAddress::L2Type&   srcAddress = af->getSrcAddr();
    NeighborList::iterator    it         = findNeighbor(srcAddress);
    const DeciderResult80211* result     = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af));
    double snr = result->getSnr();

    double bitrate = BITRATES_80211[0];
    NeighborEntry entry;

    if(snr > snrThresholds[0]) bitrate = BITRATES_80211[1];
    if(snr > snrThresholds[1]) bitrate = BITRATES_80211[2];
    if(snr > snrThresholds[2]) bitrate = BITRATES_80211[3];

    if(it != neighbors.end()) {
        it->age = simTime();
        it->bitrate = bitrate;
    }
    else {
        if(neighbors.size() < neighborhoodCacheSize) {
            entry.address = srcAddress;
            entry.age = simTime();
            entry.bitrate = bitrate;
            entry.fsc = 0;
            neighbors.push_back(entry);
        }
        else {
            it = findOldestNeighbor();
            if(it != neighbors.end()) {
                it->age = simTime();
                it->bitrate = bitrate;
                it->address = srcAddress;
                it->fsc = 0;
            }
        }
    }
    debugEV << "updated information for neighbor: " << srcAddress
       << " snr: " << snr << " bitrate: " << bitrate << endl;
}

Mac80211MultiChannel::~Mac80211MultiChannel() {
    cancelAndDelete(timeout);
    cancelAndDelete(nav);

    for (int i=0; i<13; i++) {
        cancelAndDelete(sch_nav[i]);
    }
    cancelAndDelete(reattempt);

    if(contention && !contention->isScheduled())
        delete contention;
    if(endSifs && !endSifs->isScheduled())
        delete endSifs;

    if(channelQuiet && !channelQuiet->isScheduled())
        delete channelQuiet;

    MacPktList::iterator it;
    for(it = fromUpperLayer.begin(); it != fromUpperLayer.end(); ++it) {
        delete (*it);
    }
    fromUpperLayer.clear();
}

void Mac80211MultiChannel::finish() {
    BaseMacLayer::finish();
}

