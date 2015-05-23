#ifndef __MIXIM_MULTICHANNELMAC80211_H_
#define __MIXIM_MULTICHANNELMAC80211_H_

#include <omnetpp.h>
#include <list>
#include <vector>
#include <algorithm>

#include "MiXiMDefs.h"
#include "BaseMacLayer.h"
#include "Consts80211.h"
#include "Mac80211MultiChannelPkt_m.h"

#define NO_OF_CHANNELS      3
#define LENGTH_PRA          160
#define LENGTH_PRB          LENGTH_PRA
#define LENGTH_INV          168
#define LENGTH_CFA          72
#define LENGTH_CFB          LENGTH_CFA
#define LENGTH_NCF          56
#define LENGTH_CAMACK       56


class ChannelSenseRequest;

/**
 * @brief An implementation of the 802.11b MAC.
 *
 * For more info, see the NED file.
 *
 * @ingroup macLayer
 * @ingroup ieee80211
 * @author David Raguin, Karl Wessel (port for MiXiM)
 */
class MIXIM_API Mac80211MultiChannel : public BaseMacLayer
{
private:
    /** @brief Copy constructor is not allowed.
     */
    Mac80211MultiChannel(const Mac80211MultiChannel&);
    /** @brief Assignment operator is not allowed.
     */
    Mac80211MultiChannel& operator=(const Mac80211MultiChannel&);

public:

    /** @brief frame kinds */
    enum Mac80211MultiChannelMessageKinds {
      //between MAC layers of two nodes
      RTS = LAST_BASE_MAC_MESSAGE_KIND, // request to send
      CTS,                 // clear to send
      CFA,
      CFB,
      ACK,                 // acknowledgement
      DATA,
      BROADCAST,
      OBJ,
      NCF,
      LAST_MAC_80211_MESSAGE_KIND
    };
protected:
    /** @brief Type for a queue of Mac80211MultiChannelPkts.*/
    typedef std::list<Mac80211MultiChannelPkt*> MacPktList;

    /** Definition of the timer types */
    enum timerType {
        TIMEOUT,
        REATTEMPT,
        NAV,
        SCH0_NAV,
        SCH1_NAV,
        SCH2_NAV,
        SCH3_NAV,
        SCH4_NAV,
        SCH5_NAV,
        SCH6_NAV,
        SCH7_NAV,
        SCH8_NAV,
        SCH9_NAV,
        SCH10_NAV,
        SCH11_NAV,
        SCH12_NAV,
    };

    /** Definition of the states*/
    enum State {
        WFDATA = 0, // waiting for data packet
        QUIET = 1,  // waiting for the communication between two other nodes to end
        IDLE = 2,   // no packet to send, no packet receiving
        CONTEND = 3,// contention state (battle for the channel)
        WFCTS = 4,  // RTS sent, waiting for CTS
        WFCFB = 5,
        WFACK = 6,  // DATA packet sent, waiting for ACK
        WFCFA = 7,
        BUSY = 8    // during transmission of an ACK or a BROADCAST packet
    };

    /** @brief Data about a neighbor host.*/
    struct NeighborEntry {
        /** @brief The neighbors address.*/
        LAddress::L2Type address;
        int              fsc;
        simtime_t        age;
        double           bitrate;

        NeighborEntry() : address(), fsc(0), age(), bitrate(0) {}
    };

    /** @brief Type for a list of NeighborEntries.*/
    typedef std::list<NeighborEntry> NeighborList;

  public:
    Mac80211MultiChannel();
    virtual ~Mac80211MultiChannel();

    virtual void initialize(int);
    virtual void finish();

    /**
    * @brief Tells the MAC layer to switch to the passed channel.
    *
    * This method can be used by upper layers to change the channel.
    * @param channel The channel to switch to, must be 1<=channel<=14.
    */
    void switchChannel(int channel);

    /**
    * @brief Returns the currently used channel.
    * @return The currently used channel.
    */
    int getChannel() const;

 protected:

    /** @brief Handle self messages such as timer... */
    virtual void handleSelfMsg(cMessage*);

    /** @brief Handle messages from upper layer */
    virtual void handleUpperMsg(cMessage* msg);

    /** @brief Handle messages from lower layer */
    virtual void handleLowerMsg(cMessage*);

    /** @brief Handle messages from lower layer */
    virtual void handleLowerControl(cMessage*);


    /** @brief handle end of contention */
    virtual void handleEndContentionTimer();

    bool overideSCHNAV(int sch, simtime_t duration)
    {
        if (sch_nav[sch]->getArrivalTime() < simTime() + duration)
        {
            cancelEvent(sch_nav[sch]);
            scheduleAt(simTime() + duration, sch_nav[sch]);
            return true;
        }
        return false;
    }

    simtime_t getEarliestArrivalSCHNAV(void)
    {
        simtime_t earliest = sch_nav[0]->getArrivalTime();
        int earliest_ch = 0;

        for (int i=1; i<NO_OF_CHANNELS; i++) {
            if (sch_nav[i]->getArrivalTime() < earliest) {
                earliest = sch_nav[i]->getArrivalTime();
                earliest_ch = i;
            }
        }
        // earliest can either be '0', a time in the future, or a time in the past
        if (earliest == 0 || earliest < simTime())
            earliest = simTime();

        debugEV << "earliest available service channel is Service Channel " << earliest_ch + 1 << " at " << earliest << "\n";
        return earliest;
    }

    bool availableSCH(void)
    {
        for (int i=0; i<NO_OF_CHANNELS; i++) {
            if (!SCHchannelInfo[i].occupied) return true;
        }
        debugEV << "no service channels available\n";
        return false;
    }

    void selectAvailableSCH(void)
    {
        int i;
        for (i=0; i<NO_OF_CHANNELS && SCHchannelInfo[i].occupied; i++);
        selectedSCH = i;
        debugEV << " Mac80211MultiChannel::selectAvailableSCH: Selected SCH " << i << "\n";
    }

    void cycleNextAvailableSCH(int sch)
    {
        sch++;
        if (sch >= NO_OF_CHANNELS)
            sch = 0;
        int i;
        for (i=sch; i<NO_OF_CHANNELS && SCHchannelInfo[i].occupied; i++) {
            if (i == NO_OF_CHANNELS-1) i = -1;
        }
        selectedSCH = i;
        debugEV << " Mac80211MultiChannel::cycleNextAvailableSCH: Selected SCH " << i << "\n";
    }

    int isInSCH(LAddress::L2Type address)
    {
        for (int i=0; i<NO_OF_CHANNELS; i++) {
            if ((address == SCHchannelInfo[i].receiver) ||
                (address == SCHchannelInfo[i].sender))
            {
                debugEV << "Mac80211MultiChannel::isInSCH: " << address
                        << " is in service channel " << i+1 << endl;
                return i;
            }
        }
        return -1;
    }

    /** @brief handle a message that is not for me or errornous*/
    void handleMsgNotForMe(cMessage *af, simtime_t_cref duration);
    /** @brief handle a message that was meant for me*/
    void handleMsgForMe(Mac80211MultiChannelPkt*);
    // ** @brief handle a Broadcast message*/
    void handleBroadcastMsg(Mac80211MultiChannelPkt*);

    /** @brief handle the end of a transmission...*/
    void handleEndTransmission();

    /** @brief handle end of SIFS*/
    void handleEndSifsTimer();
    /** @brief handle time out*/
    void handleTimeoutTimer();
    /** @brief NAV timer expired, the exchange of messages of other
       stations is done*/
    void handleNavTimer();

    void handleRTSframe(Mac80211MultiChannelPkt*);

    void handleDATAframe(Mac80211MultiChannelPkt*);

    void handleACKframe(Mac80211MultiChannelPkt*);

    void handleCTSframe(Mac80211MultiChannelPkt*);

    void handleCFAframe(Mac80211MultiChannelPkt*);

    void handleCFBframe(Mac80211MultiChannelPkt*);

    void handleNCFframe(Mac80211MultiChannelPkt*);

    void handleINVframe(Mac80211MultiChannelPkt*);

    void dataTransmissionFailed();

    void rtsTransmissionFailed();

    bool SCH_free(int service_channel) {
        if (sch_nav[service_channel]->isScheduled()) {
            if ((sch_nav[service_channel]->getArrivalTime() - simTime() - packetDuration(LENGTH_INV, bitrate)) > 0)
                return false;
            else
                return true;
        }
        else
            return true;
    }

    /** @brief send data frame */
    virtual void sendDATAframe(Mac80211MultiChannelPkt*);

    /** @brief send Acknoledgement */
    void sendACKframe(Mac80211MultiChannelPkt*);

    /** @brief send CTS frame */
    void sendCTSframe(Mac80211MultiChannelPkt*);

    /** @brief send RTS frame */
    virtual void sendRTSframe();

    /** @brief send broadcast frame */
    void sendBROADCASTframe();

    void sendCFAframe(Mac80211MultiChannelPkt*);

    void sendCFBframe(Mac80211MultiChannelPkt*);

    void sendNCFframe();

    void sendINVframe(Mac80211MultiChannelPkt*);

    /** @brief encapsulate packet */
    virtual macpkt_ptr_t encapsMsg(cPacket *netw);

    /** @brief decapsulate packet */
    virtual cPacket* decapsMsg(macpkt_ptr_t frame);

    /** @brief start a new contention period */
    virtual void beginNewCycle();

    /** @brief Compute a backoff value */
    simtime_t backoff(bool rtscts = true);

    /** @brief Test if maximum number of retries to transmit is exceeded */
    void testMaxAttempts();

    /** @brief return a timeOut value for a certain type of frame*/
    simtime_t timeOut(Mac80211MultiChannelMessageKinds type, double br);

    /** @brief computes the duration of a transmission over the physical channel, given a certain bitrate */
    simtime_t packetDuration(double bits, double br);

    /** @brief Produce a readable name of the given state */
    const char *stateName(State state);

    /** @brief Sets the state, and produces a log message in between */
    void setState(State state);

    /** @brief Check whether the next packet should be send with RTS/CTS */
    bool rtsCts(Mac80211MultiChannelPkt* m) {
        return m->getBitLength() - MAC80211_HEADER_LENGTH > rtsCtsThreshold;
    }

    /** @brief suspend an ongoing contention, pick it up again when the channel becomes idle */
    void suspendContention();

    /** @brief figure out at which bitrate to send to this particular destination */
    double retrieveBitrate(const LAddress::L2Type& destAddress);

    /** @brief add a new entry to the neighbor list */
    void addNeighbor(Mac80211MultiChannelPkt *af);

    /** @brief find a neighbor based on his address */
    NeighborList::iterator findNeighbor(const LAddress::L2Type& address)  {
        NeighborList::iterator it;
        for(it = neighbors.begin(); it != neighbors.end(); ++it) {
            if(it->address == address) break;
        }
        return it;
    }

    /** @brief find the oldest neighbor -- usually in order to overwrite this entry */
    NeighborList::iterator findOldestNeighbor() {
        NeighborList::iterator it = neighbors.begin();
        NeighborList::iterator oldIt = neighbors.begin();
        simtime_t age = it->age;
        for(; it != neighbors.end(); ++it) {
            if(it->age < age) {
                age = it->age;
                oldIt = it;
            }
        }
        return oldIt;
    }


    /**
     * @brief Starts a channel sense request which sense the channel for the
     * passed duration or until the channel is busy.
     *
     * Used during contend state to check if the channel is free.
     */
    void senseChannelWhileIdle(simtime_t_cref duration);

    /**
     * @brief Creates the signal to be used for a packet to be sent.
     */
    Signal* createSignal(simtime_t_cref start, simtime_t_cref length, double power, double bitrate);

protected:

    // TIMERS:

    /** @brief Timer used for time-outs after the transmission of a RTS,
       a CTS, or a DATA packet*/
    cMessage* timeout;

    /** @brief Timer used for the defer time of a node. Also called NAV :
       networks allocation vector*/
    cMessage* nav;

    cMessage* sch_nav[NO_OF_CHANNELS];

    cMessage* reattempt;

    /** @brief Used to sense if the channel is idle for contention periods*/
    ChannelSenseRequest* contention;

    /** @brief Timer used to indicate the end of a SIFS*/
    ChannelSenseRequest* endSifs;

    ChannelSenseRequest* channelQuiet;

    /** @brief Stores the the time a channel sensing started.
     * Used to calculate the quiet-time of the channel if the sensing was
     * aborted. */
    simtime_t chSenseStart;

    /** @brief Current state of the MAC*/
    State state;

    /** @brief Default bitrate
     *
     * The default bitrate must be set in the omnetpp.ini. It is used
     * whenever an auto bitrate is not appropriate, like broadcasts.
     */
    double defaultBitrate;

    /** @brief The power at which data is transmitted */
    double txPower;

    /** @brief Stores the center frequency the Mac uses. */
    double centerFreq;

    /** @brief Current bit rate at which data is transmitted */
    double bitrate;
    /** @brief Auto bit rate adaptation -- switch */
    bool autoBitrate;
    /** @brief Hold RSSI thresholds at which to change the bitrates */
    std::vector<double> snrThresholds;

    /** @brief Maximal number of packets in the queue; should be set in
       the omnetpp.ini*/
    unsigned queueLength;

    /** @brief Boolean used to know if the next packet is a broadcast packet.*/
    bool nextIsBroadcast;

    /** @brief Buffering of messages from upper layer*/
    MacPktList fromUpperLayer;

    /** @brief Number of frame transmission attempt
     *
     *  Incremented when the SHORT_RETRY_LIMIT is hit, or when an ACK
     *  or CTS is missing.
     */
    unsigned longRetryCounter;

    /** @brief Number of frame transmission attempt*/
    unsigned shortRetryCounter;

    /** @brief remaining backoff time.
     * If the backoff timer is interrupted,
     * this variable holds the remaining backoff time. */
    simtime_t remainingBackoff;

    /** @brief current IFS value (DIFS or EIFS)
     * If an error has been detected, the next backoff requires EIFS,
     * once a valid frame has been received, resets to DIFS. */
    simtime_t currentIFS;

    /** @brief Number of bits in a packet before RTS/CTS is used */
    int rtsCtsThreshold;

    /** @brief Very small value used in timer scheduling in order to avoid
       multiple changements of state in the same simulation time.*/
    simtime_t delta;

    /** @brief Keep information for this many neighbors */
    unsigned neighborhoodCacheSize;
    /** @brief Consider information in cache outdate if it is older than this */
    simtime_t neighborhoodCacheMaxAge;

    /** @brief A list of this hosts neighbors.*/
    NeighborList neighbors;

    /** take care of switchover times */
    bool switching;

    /** sequence control -- to detect duplicates*/
    int fsc;

    /** service channel occupancy info*/
    struct channelOccupants {
        LAddress::L2Type sender, receiver;
        bool occupied;
    } ;
    channelOccupants SCHchannelInfo[NO_OF_CHANNELS];

    /** selected service channel to transmit data on [0-12]*/
    int selectedSCH;
};

#endif
