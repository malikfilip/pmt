#include "ns3/command-line.h"
#include "ns3/mobility-helper.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rectangle.h"
#include "ns3/hex-grid-position-allocator.h"
#include "ns3/lora-channel.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "string.h"

using namespace ns3;
using namespace lorawan;


NS_LOG_COMPONENT_DEFINE ("ProjectExample");

//Mrezna podesavanja

int numberOfDevices = 50;
double simulationTime = 3000;
double appPeriod = 600;
double mobileNodeProbability = 0.2;
double minSpeed = 40;
double maxSpeed = 80;
double resolution = 10000;
int gatewayDistance = 8000;

int main(int argc,char *argv[]){
 //Uvezivanje mreznih parametara sa konzolom 
  CommandLine cmd;
  cmd.AddValue ("Devices", "Number of end devices included to the simulation",numberOfDevices);
  cmd.AddValue ("Radius", "The radius of the area for the simulation",radius);
  cmd.AddValue ("Time", "The time for which to simulate",simulationTime);
  cmd.AddValue ("Period", "The period in seconds to be used by periodically transmitting applications",appPeriod);
  cmd.Parse (argc, argv);
  
  int gatewayRings = 2 + (std::sqrt(2) * resolution) / (gatewayDistance);
  int numberOfGateways = 3*gatewayRings*gatewayRings-3*gatewayRings+1;
  
  
  LogComponentEnable("ProjectExample",LOG_LEVEL_ALL);
  
 //Pozicioniranje uredjaja u mrezi
 
  MobilityHelper mobilityDevices,mobilityGateways;
  
  //Uniformna raspodela uredjaja u po kvadratnom gridu,duzine strancice resolution
  
  mobilityDevices.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
   "X",PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
     ("Min", DoubleValue(-resolution),
      "Max", DoubleValue(resolution))),
   "Y", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
     ("Min", DoubleValue(-resolution),
      "Max", DoubleValue(resolution))));
      mobilityDevices.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  //Raspodela gatewaya tako da optimalno pokrivaju podrucje
  
  Ptr<HexGridPositionAllocator> hexAllocator = CreateObject<HexGridPositionAllocator> (gatewayDistance / 2);
  mobilityGateways.SetPositionAllocator(hexAllocator);
  mobilityGateways.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  //Kreiramo kanal za lora komunikaciju
   LoraPhyHelper phyHelper = LoraPhyHelper ();
   Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
   loss->SetPathLossExponent (3.76);
   loss->SetReference (1, 7.7);
   
  /*Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (0.0));
  x->SetAttribute ("Max", DoubleValue (maxRandomLoss));
  Ptr<RandomPropagationLossModel> randomLoss = CreateObject<RandomPropagationLossModel> (); 
  randomLoss->SetAttribute ("Variable", PointerValue (x));
  
  loss->SetNext (randomLoss);*/
   Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
   Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);
   
  //Kreiramo helper klase da bismo uspostavili mrezu

   phyHelper.SetChannel (channel);
   
   LorawanMacHelper macHelper = LorawanMacHelper ();
   
   LoraHelper helper = LoraHelper ();
   helper.EnablePacketTracking ();
   
  //Kreiramo gateway uredjaje
  
   NodeContainer gateways;
   gateways.Create(numberOfGateways);
   mobilityGateways.Install(gateways);
  
  //Kreiramo end device uredjaje
   
   NodeContainer endDevices;
   endDevices.Create(numberOfDevices);  
   
  //Primenjujemo prethodno kreirana pravila za pozicioniranje
   
   
   int fixedPositionNodes = double (numberOfDevices) * (1 - mobileNodeProbability);
   for (int i = 0; i < fixedPositionNodes; ++i)
    {
      mobilityDevices.Install (endDevices.Get (i));
    }
   // Install mobility model on mobile nodes
    mobilityDevices.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue (Rectangle (-resolution, resolution,
                                                                    -resolution, resolution)),
                               "Distance", DoubleValue (3000),
                               "Speed", PointerValue (CreateObjectWithAttributes<UniformRandomVariable>
                                                      ("Min", DoubleValue(minSpeed),
                                                       "Max", DoubleValue(maxSpeed))));
   for (int i = fixedPositionNodes; i < (int) endDevices.GetN (); ++i)
    {
      mobilityDevices.Install (endDevices.Get (i));
    }
   
   //Kreiramo posebne LoraNetDevice objekte
   
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen = CreateObject <LoraDeviceAddressGenerator>(nwkId,nwkAddr);
    macHelper.SetAddressGenerator (addrGen);
    phyHelper.SetDeviceType (LoraPhyHelper::ED);
    macHelper.SetDeviceType (LorawanMacHelper::ED_A);
    //macHelper.SetRegion (LorawanMacHelper::EU);
    helper.Install (phyHelper, macHelper, endDevices);
    phyHelper.SetDeviceType (LoraPhyHelper::GW);
    macHelper.SetDeviceType (LorawanMacHelper::GW);
    helper.Install (phyHelper, macHelper, gateways);
    
   //Instaliramo aplikacije na EndDevice-ovima
    Time stopTime = Seconds (simulationTime);
    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod (Seconds (appPeriod));
    appHelper.SetPacketSize (23);
    //Ptr<RandomVariableStream> rv = CreateObjectWithAttributes<UniformRandomVariable> (
    	//"Min", DoubleValue (0), "Max", DoubleValue (10));
    ApplicationContainer appContainer = appHelper.Install (endDevices); 
    
    appContainer.Start(Seconds(0));
    appContainer.Stop(stopTime);
    
   //Kreiramo mrezni server
   
    NodeContainer networkServer;
    networkServer.Create(1);
    
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGateways (gateways);
    networkServerHelper.SetEndDevices (endDevices);
    networkServerHelper.Install(networkServer);
   
   //Prosledjivanje paketa od gateway-a ka serveru
   
     ForwarderHelper forwarderHelper;
     forwarderHelper.Install(gateways);
   
   
   //Rezultati
   
   helper.EnablePeriodicDeviceStatusPrinting (endDevices, gateways, "node_data.txt", Seconds (appPeriod)); //Vremenski trenutar-Redni broj paketa - redni broj nodea - koordinate nodea - data rate u mb/s,ako je 0 onda je beskonacno, TxPower-snaga transmisije u miliwatima
   helper.EnablePeriodicPhyPerformancePrinting (gateways, "physical_layer.txt", Seconds (appPeriod));//Vremenski trenutak
   //Vremenski trenutak - redni broj gatewaya - ukupan broj paketa predvidjen za taj gateway - poslati paketi - primljeni paketi
   //broj primljenih paketa - broj interferiranih - broj ne primljenih - broj paketa za koji nema primaoca - broj paketa koje gateway nije mogao da demodulise- izgubljeni zbog snage kanala 
   helper.EnablePeriodicGlobalPerformancePrinting ("global_performance.txt", Seconds (appPeriod)); 
   
      
   
   
   //Pokretanje simulacije i animacije
   
   Simulator::Stop (stopTime);
  
   AnimationInterface anim ("project.xml");
   
   anim.SetConstantPosition (networkServer.Get(0), 1.5*resolution,0);
   
   uint32_t resourceIdNS =anim.AddResource("/home/filip/Downloads/500x500.png");
   anim.UpdateNodeImage (numberOfDevices+numberOfGateways,resourceIdNS);
   anim.UpdateNodeSize(numberOfDevices+numberOfGateways,5000,5000);
   anim.UpdateNodeDescription(numberOfDevices+numberOfGateways,"Network server");
   
   int range = numberOfGateways;
   int fixedNodes = int(fixedPositionNodes);
   int temperatureNodes = fixedNodes / 2;
   int waterNodes = fixedNodes - temperatureNodes;
   int mobileNodes = numberOfDevices-fixedNodes;
   int busNodes = mobileNodes / 2;
   int helicopterNodes = mobileNodes - busNodes;
   
   uint32_t resourceIdGW =anim.AddResource("/home/filip/Downloads/gate.png");
   for (int i = 0; i < range; ++i)
    {
      anim.UpdateNodeImage (i,resourceIdGW);
      anim.UpdateNodeSize(i,3000,3000);
      anim.UpdateNodeDescription(i,"");
    }
   
   
   uint32_t resourceIdT =anim.AddResource("/home/filip/Downloads/vatrica.png");//https://www.hiclipart.com/free-transparent-background-png-clipart-thgrb
   for (int i = range; i < (range+temperatureNodes); ++i)
    {
      anim.UpdateNodeImage (i,resourceIdT);
      anim.UpdateNodeSize(i,1000,1000);
      anim.UpdateNodeDescription(i,"");
    }
    range+=temperatureNodes;
    
    uint32_t resourceIdW =anim.AddResource("/home/filip/Downloads/vodica.png");//https://www.hiclipart.com/free-transparent-background-png-clipart-ylvpg
    for (int i = range; i < (range+waterNodes); ++i)
    {
      anim.UpdateNodeImage (i,resourceIdW);
      anim.UpdateNodeSize(i,1000,1000);
      anim.UpdateNodeDescription(i,"");
    }
    range += waterNodes;
    
    uint32_t resourceIdB =anim.AddResource("/home/filip/Downloads/bus.png");//https://www.hiclipart.com/free-transparent-background-png-clipart-ylvpg
    for (int i = range; i < (range+busNodes); ++i)
    {
      anim.UpdateNodeImage (i,resourceIdB);
      anim.UpdateNodeSize(i,2500,2500);
      anim.UpdateNodeDescription(i,"");
    }
    
    range +=busNodes;
    
    uint32_t resourceIdH =anim.AddResource("/home/filip/Downloads/helis.png");//https://www.hiclipart.com/free-transparent-background-png-clipart-ylvpg
    for (int i = range; i < (range+helicopterNodes); ++i)
    {
      anim.UpdateNodeImage (i,resourceIdH);
      anim.UpdateNodeSize(i,2500,2500);
      anim.UpdateNodeDescription(i,"");
    }
   
   
  
   
   Simulator::Run ();
   Simulator::Destroy ();
   
   NS_LOG_INFO ("Computing performance metrics...");

  LoraPacketTracker &tracker = helper.GetPacketTracker ();
  std::cout << tracker.CountMacPacketsGlobally (Seconds (0), stopTime+Hours(1)) << std::endl;
  //Prvi broj je broj poslatih paketa, drugi je broj primljenih paketa
   
    
     



 return 0;
}


