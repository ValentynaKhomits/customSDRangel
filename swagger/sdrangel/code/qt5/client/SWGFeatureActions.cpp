/**
 * SDRangel
 * This is the web REST/JSON API of SDRangel SDR software. SDRangel is an Open Source Qt5/OpenGL 3.0+ (4.3+ in Windows) GUI and server Software Defined Radio and signal analyzer in software. It supports Airspy, BladeRF, HackRF, LimeSDR, PlutoSDR, RTL-SDR, SDRplay RSP1 and FunCube    ---   Limitations and specifcities:    * In SDRangel GUI the first Rx device set cannot be deleted. Conversely the server starts with no device sets and its number of device sets can be reduced to zero by as many calls as necessary to /sdrangel/deviceset with DELETE method.   * Preset import and export from/to file is a server only feature.   * Device set focus is a GUI only feature.   * The following channels are not implemented (status 501 is returned): ATV and DATV demodulators, Channel Analyzer NG, LoRa demodulator   * The device settings and report structures contains only the sub-structure corresponding to the device type. The DeviceSettings and DeviceReport structures documented here shows all of them but only one will be or should be present at a time   * The channel settings and report structures contains only the sub-structure corresponding to the channel type. The ChannelSettings and ChannelReport structures documented here shows all of them but only one will be or should be present at a time    --- 
 *
 * OpenAPI spec version: 6.0.0
 * Contact: f4exb06@gmail.com
 *
 * NOTE: This class is auto generated by the swagger code generator program.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */


#include "SWGFeatureActions.h"

#include "SWGHelpers.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QObject>
#include <QDebug>

namespace SWGSDRangel {

SWGFeatureActions::SWGFeatureActions(QString* json) {
    init();
    this->fromJson(*json);
}

SWGFeatureActions::SWGFeatureActions() {
    feature_type = nullptr;
    m_feature_type_isSet = false;
    originator_feature_set_index = 0;
    m_originator_feature_set_index_isSet = false;
    originator_feature_index = 0;
    m_originator_feature_index_isSet = false;
    afc_actions = nullptr;
    m_afc_actions_isSet = false;
    gs232_controller_actions = nullptr;
    m_gs232_controller_actions_isSet = false;
    map_actions = nullptr;
    m_map_actions_isSet = false;
    per_tester_actions = nullptr;
    m_per_tester_actions_isSet = false;
    rig_ctl_server_actions = nullptr;
    m_rig_ctl_server_actions_isSet = false;
    satellite_tracker_actions = nullptr;
    m_satellite_tracker_actions_isSet = false;
    simple_ptt_actions = nullptr;
    m_simple_ptt_actions_isSet = false;
    star_tracker_actions = nullptr;
    m_star_tracker_actions_isSet = false;
    vor_localizer_actions = nullptr;
    m_vor_localizer_actions_isSet = false;
}

SWGFeatureActions::~SWGFeatureActions() {
    this->cleanup();
}

void
SWGFeatureActions::init() {
    feature_type = new QString("");
    m_feature_type_isSet = false;
    originator_feature_set_index = 0;
    m_originator_feature_set_index_isSet = false;
    originator_feature_index = 0;
    m_originator_feature_index_isSet = false;
    afc_actions = new SWGAFCActions();
    m_afc_actions_isSet = false;
    gs232_controller_actions = new SWGGS232ControllerActions();
    m_gs232_controller_actions_isSet = false;
    map_actions = new SWGMapActions();
    m_map_actions_isSet = false;
    per_tester_actions = new SWGPERTesterActions();
    m_per_tester_actions_isSet = false;
    rig_ctl_server_actions = new SWGRigCtlServerActions();
    m_rig_ctl_server_actions_isSet = false;
    satellite_tracker_actions = new SWGSatelliteTrackerActions();
    m_satellite_tracker_actions_isSet = false;
    simple_ptt_actions = new SWGSimplePTTActions();
    m_simple_ptt_actions_isSet = false;
    star_tracker_actions = new SWGStarTrackerActions();
    m_star_tracker_actions_isSet = false;
    vor_localizer_actions = new SWGVORLocalizerActions();
    m_vor_localizer_actions_isSet = false;
}

void
SWGFeatureActions::cleanup() {
    if(feature_type != nullptr) { 
        delete feature_type;
    }


    if(afc_actions != nullptr) { 
        delete afc_actions;
    }
    if(gs232_controller_actions != nullptr) { 
        delete gs232_controller_actions;
    }
    if(map_actions != nullptr) { 
        delete map_actions;
    }
    if(per_tester_actions != nullptr) { 
        delete per_tester_actions;
    }
    if(rig_ctl_server_actions != nullptr) { 
        delete rig_ctl_server_actions;
    }
    if(satellite_tracker_actions != nullptr) { 
        delete satellite_tracker_actions;
    }
    if(simple_ptt_actions != nullptr) { 
        delete simple_ptt_actions;
    }
    if(star_tracker_actions != nullptr) { 
        delete star_tracker_actions;
    }
    if(vor_localizer_actions != nullptr) { 
        delete vor_localizer_actions;
    }
}

SWGFeatureActions*
SWGFeatureActions::fromJson(QString &json) {
    QByteArray array (json.toStdString().c_str());
    QJsonDocument doc = QJsonDocument::fromJson(array);
    QJsonObject jsonObject = doc.object();
    this->fromJsonObject(jsonObject);
    return this;
}

void
SWGFeatureActions::fromJsonObject(QJsonObject &pJson) {
    ::SWGSDRangel::setValue(&feature_type, pJson["featureType"], "QString", "QString");
    
    ::SWGSDRangel::setValue(&originator_feature_set_index, pJson["originatorFeatureSetIndex"], "qint32", "");
    
    ::SWGSDRangel::setValue(&originator_feature_index, pJson["originatorFeatureIndex"], "qint32", "");
    
    ::SWGSDRangel::setValue(&afc_actions, pJson["AFCActions"], "SWGAFCActions", "SWGAFCActions");
    
    ::SWGSDRangel::setValue(&gs232_controller_actions, pJson["GS232ControllerActions"], "SWGGS232ControllerActions", "SWGGS232ControllerActions");
    
    ::SWGSDRangel::setValue(&map_actions, pJson["MapActions"], "SWGMapActions", "SWGMapActions");
    
    ::SWGSDRangel::setValue(&per_tester_actions, pJson["PERTesterActions"], "SWGPERTesterActions", "SWGPERTesterActions");
    
    ::SWGSDRangel::setValue(&rig_ctl_server_actions, pJson["RigCtlServerActions"], "SWGRigCtlServerActions", "SWGRigCtlServerActions");
    
    ::SWGSDRangel::setValue(&satellite_tracker_actions, pJson["SatelliteTrackerActions"], "SWGSatelliteTrackerActions", "SWGSatelliteTrackerActions");
    
    ::SWGSDRangel::setValue(&simple_ptt_actions, pJson["SimplePTTActions"], "SWGSimplePTTActions", "SWGSimplePTTActions");
    
    ::SWGSDRangel::setValue(&star_tracker_actions, pJson["StarTrackerActions"], "SWGStarTrackerActions", "SWGStarTrackerActions");
    
    ::SWGSDRangel::setValue(&vor_localizer_actions, pJson["VORLocalizerActions"], "SWGVORLocalizerActions", "SWGVORLocalizerActions");
    
}

QString
SWGFeatureActions::asJson ()
{
    QJsonObject* obj = this->asJsonObject();

    QJsonDocument doc(*obj);
    QByteArray bytes = doc.toJson();
    delete obj;
    return QString(bytes);
}

QJsonObject*
SWGFeatureActions::asJsonObject() {
    QJsonObject* obj = new QJsonObject();
    if(feature_type != nullptr && *feature_type != QString("")){
        toJsonValue(QString("featureType"), feature_type, obj, QString("QString"));
    }
    if(m_originator_feature_set_index_isSet){
        obj->insert("originatorFeatureSetIndex", QJsonValue(originator_feature_set_index));
    }
    if(m_originator_feature_index_isSet){
        obj->insert("originatorFeatureIndex", QJsonValue(originator_feature_index));
    }
    if((afc_actions != nullptr) && (afc_actions->isSet())){
        toJsonValue(QString("AFCActions"), afc_actions, obj, QString("SWGAFCActions"));
    }
    if((gs232_controller_actions != nullptr) && (gs232_controller_actions->isSet())){
        toJsonValue(QString("GS232ControllerActions"), gs232_controller_actions, obj, QString("SWGGS232ControllerActions"));
    }
    if((map_actions != nullptr) && (map_actions->isSet())){
        toJsonValue(QString("MapActions"), map_actions, obj, QString("SWGMapActions"));
    }
    if((per_tester_actions != nullptr) && (per_tester_actions->isSet())){
        toJsonValue(QString("PERTesterActions"), per_tester_actions, obj, QString("SWGPERTesterActions"));
    }
    if((rig_ctl_server_actions != nullptr) && (rig_ctl_server_actions->isSet())){
        toJsonValue(QString("RigCtlServerActions"), rig_ctl_server_actions, obj, QString("SWGRigCtlServerActions"));
    }
    if((satellite_tracker_actions != nullptr) && (satellite_tracker_actions->isSet())){
        toJsonValue(QString("SatelliteTrackerActions"), satellite_tracker_actions, obj, QString("SWGSatelliteTrackerActions"));
    }
    if((simple_ptt_actions != nullptr) && (simple_ptt_actions->isSet())){
        toJsonValue(QString("SimplePTTActions"), simple_ptt_actions, obj, QString("SWGSimplePTTActions"));
    }
    if((star_tracker_actions != nullptr) && (star_tracker_actions->isSet())){
        toJsonValue(QString("StarTrackerActions"), star_tracker_actions, obj, QString("SWGStarTrackerActions"));
    }
    if((vor_localizer_actions != nullptr) && (vor_localizer_actions->isSet())){
        toJsonValue(QString("VORLocalizerActions"), vor_localizer_actions, obj, QString("SWGVORLocalizerActions"));
    }

    return obj;
}

QString*
SWGFeatureActions::getFeatureType() {
    return feature_type;
}
void
SWGFeatureActions::setFeatureType(QString* feature_type) {
    this->feature_type = feature_type;
    this->m_feature_type_isSet = true;
}

qint32
SWGFeatureActions::getOriginatorFeatureSetIndex() {
    return originator_feature_set_index;
}
void
SWGFeatureActions::setOriginatorFeatureSetIndex(qint32 originator_feature_set_index) {
    this->originator_feature_set_index = originator_feature_set_index;
    this->m_originator_feature_set_index_isSet = true;
}

qint32
SWGFeatureActions::getOriginatorFeatureIndex() {
    return originator_feature_index;
}
void
SWGFeatureActions::setOriginatorFeatureIndex(qint32 originator_feature_index) {
    this->originator_feature_index = originator_feature_index;
    this->m_originator_feature_index_isSet = true;
}

SWGAFCActions*
SWGFeatureActions::getAfcActions() {
    return afc_actions;
}
void
SWGFeatureActions::setAfcActions(SWGAFCActions* afc_actions) {
    this->afc_actions = afc_actions;
    this->m_afc_actions_isSet = true;
}

SWGGS232ControllerActions*
SWGFeatureActions::getGs232ControllerActions() {
    return gs232_controller_actions;
}
void
SWGFeatureActions::setGs232ControllerActions(SWGGS232ControllerActions* gs232_controller_actions) {
    this->gs232_controller_actions = gs232_controller_actions;
    this->m_gs232_controller_actions_isSet = true;
}

SWGMapActions*
SWGFeatureActions::getMapActions() {
    return map_actions;
}
void
SWGFeatureActions::setMapActions(SWGMapActions* map_actions) {
    this->map_actions = map_actions;
    this->m_map_actions_isSet = true;
}

SWGPERTesterActions*
SWGFeatureActions::getPerTesterActions() {
    return per_tester_actions;
}
void
SWGFeatureActions::setPerTesterActions(SWGPERTesterActions* per_tester_actions) {
    this->per_tester_actions = per_tester_actions;
    this->m_per_tester_actions_isSet = true;
}

SWGRigCtlServerActions*
SWGFeatureActions::getRigCtlServerActions() {
    return rig_ctl_server_actions;
}
void
SWGFeatureActions::setRigCtlServerActions(SWGRigCtlServerActions* rig_ctl_server_actions) {
    this->rig_ctl_server_actions = rig_ctl_server_actions;
    this->m_rig_ctl_server_actions_isSet = true;
}

SWGSatelliteTrackerActions*
SWGFeatureActions::getSatelliteTrackerActions() {
    return satellite_tracker_actions;
}
void
SWGFeatureActions::setSatelliteTrackerActions(SWGSatelliteTrackerActions* satellite_tracker_actions) {
    this->satellite_tracker_actions = satellite_tracker_actions;
    this->m_satellite_tracker_actions_isSet = true;
}

SWGSimplePTTActions*
SWGFeatureActions::getSimplePttActions() {
    return simple_ptt_actions;
}
void
SWGFeatureActions::setSimplePttActions(SWGSimplePTTActions* simple_ptt_actions) {
    this->simple_ptt_actions = simple_ptt_actions;
    this->m_simple_ptt_actions_isSet = true;
}

SWGStarTrackerActions*
SWGFeatureActions::getStarTrackerActions() {
    return star_tracker_actions;
}
void
SWGFeatureActions::setStarTrackerActions(SWGStarTrackerActions* star_tracker_actions) {
    this->star_tracker_actions = star_tracker_actions;
    this->m_star_tracker_actions_isSet = true;
}

SWGVORLocalizerActions*
SWGFeatureActions::getVorLocalizerActions() {
    return vor_localizer_actions;
}
void
SWGFeatureActions::setVorLocalizerActions(SWGVORLocalizerActions* vor_localizer_actions) {
    this->vor_localizer_actions = vor_localizer_actions;
    this->m_vor_localizer_actions_isSet = true;
}


bool
SWGFeatureActions::isSet(){
    bool isObjectUpdated = false;
    do{
        if(feature_type && *feature_type != QString("")){
            isObjectUpdated = true; break;
        }
        if(m_originator_feature_set_index_isSet){
            isObjectUpdated = true; break;
        }
        if(m_originator_feature_index_isSet){
            isObjectUpdated = true; break;
        }
        if(afc_actions && afc_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(gs232_controller_actions && gs232_controller_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(map_actions && map_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(per_tester_actions && per_tester_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(rig_ctl_server_actions && rig_ctl_server_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(satellite_tracker_actions && satellite_tracker_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(simple_ptt_actions && simple_ptt_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(star_tracker_actions && star_tracker_actions->isSet()){
            isObjectUpdated = true; break;
        }
        if(vor_localizer_actions && vor_localizer_actions->isSet()){
            isObjectUpdated = true; break;
        }
    }while(false);
    return isObjectUpdated;
}
}

