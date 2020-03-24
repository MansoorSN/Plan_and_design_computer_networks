#include "ns3/core-module.h"
#include "ns3/propagation-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"


#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;
#define Downlink true
#define Uplink false
#define PI 3.14159265
#define PI_e5 314158

class BusStation
{
public:
    BusStation(bool downlinkUplink, std::string in_modes);
    void EnableRtsCts(bool enableCtsRts);
    void CreateNode(size_t in_ap, size_t in_nodeNumber, double radius);
    void InitialBusStation();
    void ConfigApplication(size_t in_packetSize, size_t in_dataRate);
    void Run(size_t in_simTime);
    void RxError (std::string context, Ptr<const Packet> packet,    double snr);
    void RxOk (std::string context, Ptr<const Packet> packet,double snr, WifiMode mode, enum WifiPreamble
    preamble);
    void TxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode,WifiPreamble preamble, uint8_t txPower);
	
private:
	void ConfigWifiChannel();
	void InstallDevices();
	void InstallIp();
	bool m_enableCtsRts;
	bool m_downlinkUplink;
	size_t ap_number;
	size_t node_number;
	double radius;
	size_t rx_ok;
	size_t rx_error;
	size_t tx_ok;
	std::string m_modes;
	NodeContainer nodes;
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> ap_pos;
	Ptr<ListPositionAllocator> node_pos;
	YansWifiChannelHelper wifi_Channel;
	WifiHelper m_wifi;
	YansWifiPhyHelper wifi_phy_helper;
	WifiMacHelper wifi_Mac;
	NetDeviceContainer user_devices;
	InternetStackHelper m_internet;
	Ipv4AddressHelper p_ipv4;
	ApplicationContainer m_cbrApps;
	ApplicationContainer m_pingApps;
};
BusStation::BusStation(bool in_downlinkUplink, std::string in_modes):
		m_downlinkUplink(in_downlinkUplink), m_modes(in_modes)
{
	rx_ok = 0;
	rx_error = 0;
	tx_ok = 0;
}

void BusStation::InitialBusStation()
{
	ConfigWifiChannel();
	InstallDevices();
	InstallIp();
}
void BusStation::EnableRtsCts(bool in_enableCtsRts)
{
	m_enableCtsRts = in_enableCtsRts;
	UintegerValue ctsThr = (m_enableCtsRts ? UintegerValue (10) :
	UintegerValue (22000));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
	ctsThr);
}
void BusStation::CreateNode(size_t in_ap, size_t in_nodeNumber, double in_radius)
{
	std::cout<<"range : "<<in_radius<<"\n";
	ap_number = in_ap;
	node_number = in_nodeNumber;
	radius = in_radius;
	nodes.Create(ap_number+node_number);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
	ap_pos = CreateObject<ListPositionAllocator> ();
	node_pos = CreateObject<ListPositionAllocator> ();
	for(size_t i=0; i<ap_number; ++i){
		ap_pos->Add(Vector(radius*std::cos(i*2*PI/ap_number),
		radius*std::sin(i*2*PI/ap_number), 1));
	}
	mobility.SetPositionAllocator(ap_pos);
	for(size_t i=0; i<ap_number; ++i){
		mobility.Install(nodes.Get(i));
	}
	for(size_t i=0; i<node_number; ++i){
		size_t inAp = i/(node_number/ap_number);
		double nodeRadius = rand()%120+(rand()%1000)/1000;
		node_pos->Add(Vector(radius*std::cos(inAp*2*PI/ap_number)+
		nodeRadius*std::cos((rand()%(2*PI_e5))/pow(10,5)),radius*std::sin(inAp*2*PI/ap_number)+nodeRadius*std::sin((rand()%(2*PI_e5))/pow(10,5)),1));
	}
	mobility.SetPositionAllocator(node_pos);
	for(size_t i=0; i<node_number; ++i){
		mobility.Install(nodes.Get(ap_number+i));
	}
}
void  BusStation::ConfigWifiChannel()
{
	wifi_Channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
wifi_Channel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel","Frequency", DoubleValue(2.400e9));
}
void BusStation::InstallDevices()
{
	m_wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue("DsssRate2Mbps"));
	m_wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode",StringValue (m_modes),"ControlMode",StringValue (m_modes));
	wifi_phy_helper = YansWifiPhyHelper::Default ();
	wifi_phy_helper.SetChannel (wifi_Channel.Create());
	wifi_phy_helper.Set ("EnergyDetectionThreshold", DoubleValue (-95.0) );
	wifi_phy_helper.Set ("CcaEdThreshold", DoubleValue (-95.0) );
	wifi_phy_helper.Set ("TxPowerStart", DoubleValue (23.0) );
	wifi_phy_helper.Set ("TxPowerEnd", DoubleValue (23.0) );
	wifi_phy_helper.Set ("ChannelNumber", UintegerValue (1) );
	wifi_phy_helper.Set ("RxGain", DoubleValue (-25.0));
	WifiMacHelper wifi_Mac;
	wifi_Mac.SetType ("ns3::AdhocWifiMac"); // use ad-hoc MAC
	user_devices = m_wifi.Install (wifi_phy_helper, wifi_Mac, nodes);
}
void BusStation::InstallIp()
{
	m_internet.Install (nodes);
	p_ipv4.SetBase ("10.0.0.0", "255.0.0.0");
	p_ipv4.Assign (user_devices);
}
void BusStation::RxError (std::string context, Ptr<const Packet>packet, double snr)
{
	Ptr<Packet> m_currentPacket;
	WifiMacHeader hdr;
	m_currentPacket = packet->Copy();
	m_currentPacket->RemoveHeader (hdr);
	if(hdr.IsData()){
		rx_error++;
	}
}
void BusStation::RxOk (std::string context, Ptr<const Packet>packet,double snr, WifiMode mode, enum WifiPreamble preamble)
{
	Ptr<Packet> m_currentPacket;
	WifiMacHeader hdr;
	m_currentPacket = packet->Copy();
	m_currentPacket->RemoveHeader (hdr);
	if(hdr.IsData()){
		rx_ok++;
	}
}
void BusStation::TxTrace (std::string context, Ptr<const Packet> packet,WifiMode mode, WifiPreamble preamble, uint8_t
	txPower)
{
	Ptr<Packet> m_currentPacket;
	WifiMacHeader hdr;
	m_currentPacket = packet->Copy();
	m_currentPacket->RemoveHeader (hdr);
	if(hdr.IsData()){
		tx_ok++;
	}
}
void BusStation::ConfigApplication(size_t in_packetSize, size_t in_dataRate)
{
	uint16_t cbrPort = 12345;
	for(size_t j=1; j<=ap_number; ++j){
		for(size_t i=ap_number+node_number/ap_number*(j-1);i<ap_number+node_number/ap_number*j ; ++i){
			std::string s;
			std::stringstream ss(s);
			if(m_downlinkUplink){
				ss << i+1;
			}else
			{
				ss << j;
			}
			s = "10.0.0."+ss.str();
			OnOffHelper onOffHelper ("ns3::UdpSocketFactory",InetSocketAddress (Ipv4Address (s.c_str()), cbrPort));
			onOffHelper.SetAttribute ("PacketSize", UintegerValue(in_packetSize));
			onOffHelper.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
			onOffHelper.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
			std::string s2;
			std::stringstream ss2(s2);
			if(m_downlinkUplink){
				ss2 << in_dataRate+i*100;
			}else
			{
				ss2 << in_dataRate+i*100;
			}
			s2 = ss2.str() + "bps";
			onOffHelper.SetAttribute ("DataRate", StringValue (s2));
			if(m_downlinkUplink){
				onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds(1.00+static_cast<double>(i)/100)));
				onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds(50.000+static_cast<double>(i)/100)));
				m_cbrApps.Add (onOffHelper.Install (nodes.Get (j-1)));
			}else
			{
				onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds(1.00)));
				onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds(50.000+static_cast<double>(j)/100)));
				m_cbrApps.Add (onOffHelper.Install (nodes.Get (i)));
			}
		}
	}
	uint16_t echoPort = 9;
// again using different start times to workaround Bug 388 and Bug 912
	for(size_t j=1; j<=ap_number; ++j){
		for(size_t i=ap_number+node_number/ap_number*(j-1);i<ap_number+node_number/ap_number*j ; ++i){
			std::string s;
			std::stringstream ss(s);
			if(m_downlinkUplink){
				ss << i+1;
			}else
			{
				ss << j;
			}
			s = "10.0.0."+ss.str();
//std::cout<<"hi";
			UdpEchoClientHelper echoClientHelper (Ipv4Address (s.c_str()),echoPort);
			echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
			echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds(0.1)));
			echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
			if(m_downlinkUplink){
				echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds(0.001)));
				echoClientHelper.SetAttribute ("StopTime", TimeValue (Seconds(50.001)));
				m_pingApps.Add (echoClientHelper.Install (nodes.Get (j-1)));
			}else
			{
				echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds(0.001)));
				echoClientHelper.SetAttribute ("StopTime", TimeValue (Seconds(50.001)));
				m_pingApps.Add (echoClientHelper.Install (nodes.Get (i)));
			}
		}
	}
}

void BusStation::Run(size_t in_simTime)
{
	// 8. Install FlowMonitor on all nodes
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
	// 9. Run simulation
	Simulator::Stop (Seconds (in_simTime));
	Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError",	MakeCallback (&BusStation::RxError, this));
	Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk",	MakeCallback (&BusStation::RxOk, this));
	Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx",	MakeCallback (&BusStation::TxTrace, this));
	Simulator::Run ();
	// 10. Print per flow statistics
	monitor->CheckForLostPackets ();
	Ptr<Ipv4FlowClassifier> classifier =DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
	std::map<FlowId, FlowMonitor::FlowStats> stats =	monitor->GetFlowStats ();
	double accumulatedThroughput = 0;
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator
	i=stats.begin();
	i!=stats.end(); ++i)
	{
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
		std::cout << "Flow " << i->first<< " (" << t.sourceAddress << " ->" << t.destinationAddress << ")\n";
		
		std::cout << " Throughput: " << i->second.rxBytes * 8.0 /	in_simTime / 1024/1024 << " Mbps\n";
		accumulatedThroughput+=(i->second.rxBytes*8.0/in_simTime/1024/1024);
	}
	std::cout << "apNumber=" <<ap_number << " nodeNumber=" <<node_number << "\n" << std::flush;
	std::cout << "throughput=" << accumulatedThroughput << "\n" <<std::flush;
	std::cout << "tx=" << tx_ok << " RXerror=" <<rx_error <<" Rxok=" << rx_ok << "\n" << std::flush;
	std::cout << "===========================\n" << std::flush;
	// 11. Cleanup
	/*AnimationInterface anim("anim2.xml");
	for(size_t i=0; i< ap_number+node_number; ++i){
		Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
		Vector nodePos = mobility->GetPosition ();
		std::cout<<"nodes"<<nodePos.x<<" "<<nodePos.y<<"\n";
		anim.SetConstantPosition(nodes.Get(i), nodePos.x, nodePos.y);

	}*/

	Simulator::Destroy ();
}
int main (int argc, char **argv)
{
	size_t numOfAp[6] = {1, 2, 3, 4, 5};
	double range[4] = {60, 120};
	std::vector <std::string> modes;
	modes.push_back ("DsssRate5_5Mbps");
	std::cout << "Bus station BusStation :\n" <<std::flush;
	for(size_t i=0; i<2; ++i){
		for(size_t m=2;m<3;++m){
		for(size_t j=0; j<1; ++j){
				std::cout << "Range=" << range[j] << ", Mode=" << modes[0] <<"\n";
				BusStation exp(Downlink, modes[0]);
				exp.EnableRtsCts(false);
				exp.CreateNode(numOfAp[i], m, range[j]);
				exp.InitialBusStation();
				exp.ConfigApplication(1024, 5500000);
				exp.Run(60);
		}
		}
	}

return 0;
}

