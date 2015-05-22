//
// Generated file, do not edit! Created by opp_msgc 4.3 from modules/messages/Mac80211MultiChannelPkt.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "Mac80211MultiChannelPkt_m.h"

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}




Register_Class(Mac80211MultiChannelPkt);

Mac80211MultiChannelPkt::Mac80211MultiChannelPkt(const char *name, int kind) : MacPkt(name,kind)
{
    this->address3_var = 0;
    this->address4_var = 0;
    this->fragmentation_var = 0;
    this->informationDS_var = 0;
    this->sequenceControl_var = 0;
    this->retry_var = 0;
    this->duration_var = 0;
    this->selectedSch_var = 0;
    this->schDuration_var = 0;
}

Mac80211MultiChannelPkt::Mac80211MultiChannelPkt(const Mac80211MultiChannelPkt& other) : MacPkt(other)
{
    copy(other);
}

Mac80211MultiChannelPkt::~Mac80211MultiChannelPkt()
{
}

Mac80211MultiChannelPkt& Mac80211MultiChannelPkt::operator=(const Mac80211MultiChannelPkt& other)
{
    if (this==&other) return *this;
    MacPkt::operator=(other);
    copy(other);
    return *this;
}

void Mac80211MultiChannelPkt::copy(const Mac80211MultiChannelPkt& other)
{
    this->address3_var = other.address3_var;
    this->address4_var = other.address4_var;
    this->fragmentation_var = other.fragmentation_var;
    this->informationDS_var = other.informationDS_var;
    this->sequenceControl_var = other.sequenceControl_var;
    this->retry_var = other.retry_var;
    this->duration_var = other.duration_var;
    this->schSender_var = other.schSender_var;
    this->schReceiver_var = other.schReceiver_var;
    this->selectedSch_var = other.selectedSch_var;
    this->schDuration_var = other.schDuration_var;
}

void Mac80211MultiChannelPkt::parsimPack(cCommBuffer *b)
{
    MacPkt::parsimPack(b);
    doPacking(b,this->address3_var);
    doPacking(b,this->address4_var);
    doPacking(b,this->fragmentation_var);
    doPacking(b,this->informationDS_var);
    doPacking(b,this->sequenceControl_var);
    doPacking(b,this->retry_var);
    doPacking(b,this->duration_var);
    doPacking(b,this->schSender_var);
    doPacking(b,this->schReceiver_var);
    doPacking(b,this->selectedSch_var);
    doPacking(b,this->schDuration_var);
}

void Mac80211MultiChannelPkt::parsimUnpack(cCommBuffer *b)
{
    MacPkt::parsimUnpack(b);
    doUnpacking(b,this->address3_var);
    doUnpacking(b,this->address4_var);
    doUnpacking(b,this->fragmentation_var);
    doUnpacking(b,this->informationDS_var);
    doUnpacking(b,this->sequenceControl_var);
    doUnpacking(b,this->retry_var);
    doUnpacking(b,this->duration_var);
    doUnpacking(b,this->schSender_var);
    doUnpacking(b,this->schReceiver_var);
    doUnpacking(b,this->selectedSch_var);
    doUnpacking(b,this->schDuration_var);
}

int Mac80211MultiChannelPkt::getAddress3() const
{
    return address3_var;
}

void Mac80211MultiChannelPkt::setAddress3(int address3)
{
    this->address3_var = address3;
}

int Mac80211MultiChannelPkt::getAddress4() const
{
    return address4_var;
}

void Mac80211MultiChannelPkt::setAddress4(int address4)
{
    this->address4_var = address4;
}

int Mac80211MultiChannelPkt::getFragmentation() const
{
    return fragmentation_var;
}

void Mac80211MultiChannelPkt::setFragmentation(int fragmentation)
{
    this->fragmentation_var = fragmentation;
}

int Mac80211MultiChannelPkt::getInformationDS() const
{
    return informationDS_var;
}

void Mac80211MultiChannelPkt::setInformationDS(int informationDS)
{
    this->informationDS_var = informationDS;
}

int Mac80211MultiChannelPkt::getSequenceControl() const
{
    return sequenceControl_var;
}

void Mac80211MultiChannelPkt::setSequenceControl(int sequenceControl)
{
    this->sequenceControl_var = sequenceControl;
}

bool Mac80211MultiChannelPkt::getRetry() const
{
    return retry_var;
}

void Mac80211MultiChannelPkt::setRetry(bool retry)
{
    this->retry_var = retry;
}

simtime_t Mac80211MultiChannelPkt::getDuration() const
{
    return duration_var;
}

void Mac80211MultiChannelPkt::setDuration(simtime_t duration)
{
    this->duration_var = duration;
}

LAddress::L2Type& Mac80211MultiChannelPkt::getSchSender()
{
    return schSender_var;
}

void Mac80211MultiChannelPkt::setSchSender(const LAddress::L2Type& schSender)
{
    this->schSender_var = schSender;
}

LAddress::L2Type& Mac80211MultiChannelPkt::getSchReceiver()
{
    return schReceiver_var;
}

void Mac80211MultiChannelPkt::setSchReceiver(const LAddress::L2Type& schReceiver)
{
    this->schReceiver_var = schReceiver;
}

int Mac80211MultiChannelPkt::getSelectedSch() const
{
    return selectedSch_var;
}

void Mac80211MultiChannelPkt::setSelectedSch(int selectedSch)
{
    this->selectedSch_var = selectedSch;
}

simtime_t Mac80211MultiChannelPkt::getSchDuration() const
{
    return schDuration_var;
}

void Mac80211MultiChannelPkt::setSchDuration(simtime_t schDuration)
{
    this->schDuration_var = schDuration;
}

class Mac80211MultiChannelPktDescriptor : public cClassDescriptor
{
  public:
    Mac80211MultiChannelPktDescriptor();
    virtual ~Mac80211MultiChannelPktDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(Mac80211MultiChannelPktDescriptor);

Mac80211MultiChannelPktDescriptor::Mac80211MultiChannelPktDescriptor() : cClassDescriptor("Mac80211MultiChannelPkt", "MacPkt")
{
}

Mac80211MultiChannelPktDescriptor::~Mac80211MultiChannelPktDescriptor()
{
}

bool Mac80211MultiChannelPktDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<Mac80211MultiChannelPkt *>(obj)!=NULL;
}

const char *Mac80211MultiChannelPktDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int Mac80211MultiChannelPktDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 11+basedesc->getFieldCount(object) : 11;
}

unsigned int Mac80211MultiChannelPktDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<11) ? fieldTypeFlags[field] : 0;
}

const char *Mac80211MultiChannelPktDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "address3",
        "address4",
        "fragmentation",
        "informationDS",
        "sequenceControl",
        "retry",
        "duration",
        "schSender",
        "schReceiver",
        "selectedSch",
        "schDuration",
    };
    return (field>=0 && field<11) ? fieldNames[field] : NULL;
}

int Mac80211MultiChannelPktDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='a' && strcmp(fieldName, "address3")==0) return base+0;
    if (fieldName[0]=='a' && strcmp(fieldName, "address4")==0) return base+1;
    if (fieldName[0]=='f' && strcmp(fieldName, "fragmentation")==0) return base+2;
    if (fieldName[0]=='i' && strcmp(fieldName, "informationDS")==0) return base+3;
    if (fieldName[0]=='s' && strcmp(fieldName, "sequenceControl")==0) return base+4;
    if (fieldName[0]=='r' && strcmp(fieldName, "retry")==0) return base+5;
    if (fieldName[0]=='d' && strcmp(fieldName, "duration")==0) return base+6;
    if (fieldName[0]=='s' && strcmp(fieldName, "schSender")==0) return base+7;
    if (fieldName[0]=='s' && strcmp(fieldName, "schReceiver")==0) return base+8;
    if (fieldName[0]=='s' && strcmp(fieldName, "selectedSch")==0) return base+9;
    if (fieldName[0]=='s' && strcmp(fieldName, "schDuration")==0) return base+10;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *Mac80211MultiChannelPktDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
        "int",
        "int",
        "bool",
        "simtime_t",
        "LAddress::L2Type",
        "LAddress::L2Type",
        "int",
        "simtime_t",
    };
    return (field>=0 && field<11) ? fieldTypeStrings[field] : NULL;
}

const char *Mac80211MultiChannelPktDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    }
}

int Mac80211MultiChannelPktDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    Mac80211MultiChannelPkt *pp = (Mac80211MultiChannelPkt *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string Mac80211MultiChannelPktDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    Mac80211MultiChannelPkt *pp = (Mac80211MultiChannelPkt *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getAddress3());
        case 1: return long2string(pp->getAddress4());
        case 2: return long2string(pp->getFragmentation());
        case 3: return long2string(pp->getInformationDS());
        case 4: return long2string(pp->getSequenceControl());
        case 5: return bool2string(pp->getRetry());
        case 6: return double2string(pp->getDuration());
        case 7: {std::stringstream out; out << pp->getSchSender(); return out.str();}
        case 8: {std::stringstream out; out << pp->getSchReceiver(); return out.str();}
        case 9: return long2string(pp->getSelectedSch());
        case 10: return double2string(pp->getSchDuration());
        default: return "";
    }
}

bool Mac80211MultiChannelPktDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    Mac80211MultiChannelPkt *pp = (Mac80211MultiChannelPkt *)object; (void)pp;
    switch (field) {
        case 0: pp->setAddress3(string2long(value)); return true;
        case 1: pp->setAddress4(string2long(value)); return true;
        case 2: pp->setFragmentation(string2long(value)); return true;
        case 3: pp->setInformationDS(string2long(value)); return true;
        case 4: pp->setSequenceControl(string2long(value)); return true;
        case 5: pp->setRetry(string2bool(value)); return true;
        case 6: pp->setDuration(string2double(value)); return true;
        case 9: pp->setSelectedSch(string2long(value)); return true;
        case 10: pp->setSchDuration(string2double(value)); return true;
        default: return false;
    }
}

const char *Mac80211MultiChannelPktDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldStructNames[] = {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        "LAddress::L2Type",
        "LAddress::L2Type",
        NULL,
        NULL,
    };
    return (field>=0 && field<11) ? fieldStructNames[field] : NULL;
}

void *Mac80211MultiChannelPktDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    Mac80211MultiChannelPkt *pp = (Mac80211MultiChannelPkt *)object; (void)pp;
    switch (field) {
        case 7: return (void *)(&pp->getSchSender()); break;
        case 8: return (void *)(&pp->getSchReceiver()); break;
        default: return NULL;
    }
}


