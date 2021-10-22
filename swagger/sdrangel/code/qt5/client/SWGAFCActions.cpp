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


#include "SWGAFCActions.h"

#include "SWGHelpers.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QObject>
#include <QDebug>

namespace SWGSDRangel {

SWGAFCActions::SWGAFCActions(QString* json) {
    init();
    this->fromJson(*json);
}

SWGAFCActions::SWGAFCActions() {
    run = 0;
    m_run_isSet = false;
    device_track = 0;
    m_device_track_isSet = false;
    devices_apply = 0;
    m_devices_apply_isSet = false;
}

SWGAFCActions::~SWGAFCActions() {
    this->cleanup();
}

void
SWGAFCActions::init() {
    run = 0;
    m_run_isSet = false;
    device_track = 0;
    m_device_track_isSet = false;
    devices_apply = 0;
    m_devices_apply_isSet = false;
}

void
SWGAFCActions::cleanup() {



}

SWGAFCActions*
SWGAFCActions::fromJson(QString &json) {
    QByteArray array (json.toStdString().c_str());
    QJsonDocument doc = QJsonDocument::fromJson(array);
    QJsonObject jsonObject = doc.object();
    this->fromJsonObject(jsonObject);
    return this;
}

void
SWGAFCActions::fromJsonObject(QJsonObject &pJson) {
    ::SWGSDRangel::setValue(&run, pJson["run"], "qint32", "");
    
    ::SWGSDRangel::setValue(&device_track, pJson["deviceTrack"], "qint32", "");
    
    ::SWGSDRangel::setValue(&devices_apply, pJson["devicesApply"], "qint32", "");
    
}

QString
SWGAFCActions::asJson ()
{
    QJsonObject* obj = this->asJsonObject();

    QJsonDocument doc(*obj);
    QByteArray bytes = doc.toJson();
    delete obj;
    return QString(bytes);
}

QJsonObject*
SWGAFCActions::asJsonObject() {
    QJsonObject* obj = new QJsonObject();
    if(m_run_isSet){
        obj->insert("run", QJsonValue(run));
    }
    if(m_device_track_isSet){
        obj->insert("deviceTrack", QJsonValue(device_track));
    }
    if(m_devices_apply_isSet){
        obj->insert("devicesApply", QJsonValue(devices_apply));
    }

    return obj;
}

qint32
SWGAFCActions::getRun() {
    return run;
}
void
SWGAFCActions::setRun(qint32 run) {
    this->run = run;
    this->m_run_isSet = true;
}

qint32
SWGAFCActions::getDeviceTrack() {
    return device_track;
}
void
SWGAFCActions::setDeviceTrack(qint32 device_track) {
    this->device_track = device_track;
    this->m_device_track_isSet = true;
}

qint32
SWGAFCActions::getDevicesApply() {
    return devices_apply;
}
void
SWGAFCActions::setDevicesApply(qint32 devices_apply) {
    this->devices_apply = devices_apply;
    this->m_devices_apply_isSet = true;
}


bool
SWGAFCActions::isSet(){
    bool isObjectUpdated = false;
    do{
        if(m_run_isSet){
            isObjectUpdated = true; break;
        }
        if(m_device_track_isSet){
            isObjectUpdated = true; break;
        }
        if(m_devices_apply_isSet){
            isObjectUpdated = true; break;
        }
    }while(false);
    return isObjectUpdated;
}
}

