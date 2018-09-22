/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

//otherwise cDOM will cause a deprecated warning, however there is no alternative yet
#define A_UTILS_NO_DEPRECATED_WARNING

#include "DriverModuleFilter.h"
#include "DriverModuleWidget.h"
#include <aadc_structs.h>


#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages


ADTF_PLUGIN(LABEL_CAR_CONTROLLER, DriverModule)

using namespace aadc::jury;

DriverModule::DriverModule() : m_pWidget(nullptr)
{
    //Register Properties
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    //Get Media Descriptions
    object_ptr<IStreamType> pTypeJuryStruct;
    cString structName = "tJuryStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeJuryStruct, m_juryStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    create_pin(*this, m_oInputJuryStruct, "jury_struct", pTypeJuryStruct);
    
    object_ptr<IStreamType> pTypeDriverStruct;
    structName = "tDriverStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeDriverStruct, m_driverStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    create_pin(*this, m_oOutputDriverStruct, "driver_struct", pTypeDriverStruct);

    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    create_pin(*this, m_oInputManeuverList, "maneuver_list", pTypeDefault);
}

DriverModule::~DriverModule()
{
}

QWidget* DriverModule::CreateView()
{
    // use single UI File in background
    m_pWidget = new DisplayWidgetDriver(nullptr);

    qRegisterMetaType<aadc::jury::stateCar>("aadc::jury::stateCar");
    qRegisterMetaType<tInt16>("tInt16");

    tBool allConected = tTrue;
    allConected &= tBool(connect(m_pWidget, SIGNAL(sendStruct(aadc::jury::stateCar, tInt16)), this, SLOT(OnSendState(aadc::jury::stateCar, tInt16))));
    allConected &= tBool(connect(this, SIGNAL(SendRun(int)), m_pWidget, SLOT(OnDriverGo(int))));
    allConected &= tBool(connect(this, SIGNAL(SendStop(int)), m_pWidget, SLOT(OnDriverStop(int))));
    allConected &= tBool(connect(this, SIGNAL(SendRequestReady(int)), m_pWidget, SLOT(OnDriverRequestReady(int))));
    allConected &= tBool(connect(this, SIGNAL(TriggerLoadManeuverList()), this, SLOT(LoadManeuverList())));

    if (!allConected)
    {
        LOG_WARNING("DriverModule - not all Signal could be connected with an slot");
    }

    // disable maneuver group box until receiving maneuver list
    m_pWidget->EnableManeuverGroupBox(false);
    return m_pWidget;
}

tVoid DriverModule::ReleaseView()
{
    delete m_pWidget;
    m_pWidget = nullptr;
}

tResult DriverModule::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult DriverModule::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult DriverModule::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    return cQtUIFilter::Shutdown(eStage);
}

tResult DriverModule::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);
       
    object_ptr<const ISample> pSample;
    while (IS_OK(m_oInputJuryStruct.GetNextSample(pSample)))
    {
        auto oDecoder = m_juryStructSampleFactory.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());      
        tJuryStruct juryInput;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.maneuverEntry, &juryInput.i16ManeuverEntry));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.actionId, &juryInput.i16ActionID));
        
        tInt8 i8ActionID = juryInput.i16ActionID;
        tInt16 i16entry = juryInput.i16ManeuverEntry;

        switch (aadc::jury::juryAction(i8ActionID))
        {
            case action_getready:
                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d", i16entry));
                emit SendRequestReady(static_cast<int>(i16entry));
                break;
            case action_start:
                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d", i16entry));
                emit SendRun(static_cast<int>(i16entry));
                break;
            case action_stop:
                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d", i16entry));
                emit SendStop(static_cast<int>(i16entry));
                break;
        }
    }

    object_ptr<const ISample> pSampleAnonymous;
    while (IS_OK(m_oInputManeuverList.GetNextSample(pSampleAnonymous)))
    {

        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//maneuverlist
            m_strManeuverFileString.Set(data.data(), data.size());
            TriggerLoadManeuverList();
        }
    }
    RETURN_NOERROR;
}

tResult DriverModule::OnSendState(stateCar stateID, tInt16 i16ManeuverEntry)
{
   
    tDriverStruct driverStruct;
    driverStruct.i16StateID = stateID;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;

    if (m_propEnableConsoleOutput)
    {
        switch (stateID)
        {
            case statecar_ready:
                LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_running:
                LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_complete:
                LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_error:
                LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_startup:
                LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d", i16ManeuverEntry));
                break;
        }
    }
    RETURN_IF_FAILED(TransmitDriverStruct(driverStruct));

    RETURN_NOERROR;
}

tResult DriverModule::TransmitDriverStruct(tDriverStruct& driverStruct)
{
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_driverStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, driverStruct.i16ManeuverEntry));
    }

    m_oOutputDriverStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}


tResult DriverModule::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    sector.sector.push_back(man);
                }
            }

            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        m_pWidget->EnableManeuverGroupBox(true);
    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        m_pWidget->EnableManeuverGroupBox(false);
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    // update the ui
    m_pWidget->SetManeuverList(m_sectorList);
    m_pWidget->ShowManeuverList();
    m_pWidget->FillComboBox();


    RETURN_NOERROR;
}
