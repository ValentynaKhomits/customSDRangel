///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 F4HKW                                                      //
// for F4EXB / SDRAngel                                                          //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef DATVIDEOSTREAM_H
#define DATVIDEOSTREAM_H

#include <QIODevice>
#include <QQueue>
#include <QByteArray>
#include <QEventLoop>
#include <QMutex>
#include <QThread>

class DATVideostream : public QIODevice
{
    Q_OBJECT

public:
    DATVideostream();
    ~DATVideostream();

    static const int m_defaultMemoryLimit = 2820000;
    static const int m_minStackSize = 4;

    int pushData(const char * chrData, int intSize);
    void resetTotalReceived();
    void cleanUp();
    void setMultiThreaded(bool multiThreaded);
    void setThreadTimeout(int timeOut) { m_threadTimeout = timeOut; }

    virtual bool isSequential() const;
    virtual qint64 bytesAvailable() const;
    virtual void close();
    virtual bool open(OpenMode mode);

    QQueue<QByteArray> m_objFIFO;

signals:

    void onDataAvailable();
    void onDataPackets(int *intDataBytes, int *intPercentBuffer,qint64 *intTotalReceived);

protected:

    virtual qint64 readData(char *data, qint64 len);
    virtual qint64 writeData(const char *data, qint64 len);
    virtual qint64 readLineData(char *data, qint64 maxSize);

private:
    bool m_multiThreaded;
    int m_threadTimeout;

    QEventLoop m_objeventLoop;
    QMutex m_objMutex;
    int m_intMemoryLimit;
    int m_intBytesAvailable;
    int m_intBytesWaiting;
    int m_intPercentBuffer;
    qint64 m_intTotalReceived;
    qint64 m_intPacketReceived;

};

#endif // DATVIDEOSTREAM_H
