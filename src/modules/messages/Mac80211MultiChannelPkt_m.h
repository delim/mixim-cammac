//
// Generated file, do not edit! Created by opp_msgc 4.3 from modules/messages/Mac80211MultiChannelPkt.msg.
//

#ifndef _MAC80211MULTICHANNELPKT_M_H_
#define _MAC80211MULTICHANNELPKT_M_H_

#include <omnetpp.h>

// opp_msgc version check
#define MSGC_VERSION 0x0403
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of opp_msgc: 'make clean' should help.
#endif

// cplusplus {{
#include "MiXiMMacPkt.h"
#include "SimpleAddress.h"
// }}



/**
 * Class generated from <tt>modules/messages/Mac80211MultiChannelPkt.msg</tt> by opp_msgc.
 * <pre>
 * packet Mac80211MultiChannelPkt extends MacPkt
 * {
 *     int address3;
 *     int address4;
 *     int fragmentation; 
 *     int informationDS; 
 *     int sequenceControl;
 *     bool retry;
 *     simtime_t duration; 	
 * 						
 * 						
 * 	LAddress::L2Type schSender;
 * 	LAddress::L2Type schReceiver;
 * 	int selectedSch;
 * 	simtime_t schDuration;
 * }
 * </pre>
 */
class Mac80211MultiChannelPkt : public ::MacPkt
{
  protected:
    int address3_var;
    int address4_var;
    int fragmentation_var;
    int informationDS_var;
    int sequenceControl_var;
    bool retry_var;
    simtime_t duration_var;
    LAddress::L2Type schSender_var;
    LAddress::L2Type schReceiver_var;
    int selectedSch_var;
    simtime_t schDuration_var;

  private:
    void copy(const Mac80211MultiChannelPkt& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const Mac80211MultiChannelPkt&);

  public:
    Mac80211MultiChannelPkt(const char *name=NULL, int kind=0);
    Mac80211MultiChannelPkt(const Mac80211MultiChannelPkt& other);
    virtual ~Mac80211MultiChannelPkt();
    Mac80211MultiChannelPkt& operator=(const Mac80211MultiChannelPkt& other);
    virtual Mac80211MultiChannelPkt *dup() const {return new Mac80211MultiChannelPkt(*this);}
    virtual void parsimPack(cCommBuffer *b);
    virtual void parsimUnpack(cCommBuffer *b);

    // field getter/setter methods
    virtual int getAddress3() const;
    virtual void setAddress3(int address3);
    virtual int getAddress4() const;
    virtual void setAddress4(int address4);
    virtual int getFragmentation() const;
    virtual void setFragmentation(int fragmentation);
    virtual int getInformationDS() const;
    virtual void setInformationDS(int informationDS);
    virtual int getSequenceControl() const;
    virtual void setSequenceControl(int sequenceControl);
    virtual bool getRetry() const;
    virtual void setRetry(bool retry);
    virtual simtime_t getDuration() const;
    virtual void setDuration(simtime_t duration);
    virtual LAddress::L2Type& getSchSender();
    virtual const LAddress::L2Type& getSchSender() const {return const_cast<Mac80211MultiChannelPkt*>(this)->getSchSender();}
    virtual void setSchSender(const LAddress::L2Type& schSender);
    virtual LAddress::L2Type& getSchReceiver();
    virtual const LAddress::L2Type& getSchReceiver() const {return const_cast<Mac80211MultiChannelPkt*>(this)->getSchReceiver();}
    virtual void setSchReceiver(const LAddress::L2Type& schReceiver);
    virtual int getSelectedSch() const;
    virtual void setSelectedSch(int selectedSch);
    virtual simtime_t getSchDuration() const;
    virtual void setSchDuration(simtime_t schDuration);
};

inline void doPacking(cCommBuffer *b, Mac80211MultiChannelPkt& obj) {obj.parsimPack(b);}
inline void doUnpacking(cCommBuffer *b, Mac80211MultiChannelPkt& obj) {obj.parsimUnpack(b);}


#endif // _MAC80211MULTICHANNELPKT_M_H_
