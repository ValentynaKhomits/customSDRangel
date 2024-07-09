///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012 maintech GmbH, Otto-Hahn-Str. 15, 97204 Hoechberg, Germany //
// written by Christian Daniel                                                   //
// Copyright (C) 2015-2020, 2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com>    //
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

#include <QStringList>

#include "channeladddialog.h"
#include "ui_channeladddialog.h"

ChannelAddDialog::ChannelAddDialog(QWidget* parent) :
    QDialog(parent),
    ui(new Ui::ChannelAddDialog)
{
    ui->setupUi(this);
    connect(ui->buttonBox, SIGNAL(clicked(QAbstractButton*)), this, SLOT(apply(QAbstractButton*)));
}

ChannelAddDialog::~ChannelAddDialog()
{
    delete ui;
}

void ChannelAddDialog::resetChannelNames()
{
    ui->channelSelect->clear();
}

void ChannelAddDialog::addChannelNames(const QStringList& channelNames)
{
    ui->channelSelect->addItems(channelNames);
}

void ChannelAddDialog::apply(QAbstractButton *button)
{
        emit(addChannel(0));
}
