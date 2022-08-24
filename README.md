"# HBS_EthicalIntelligence" 


#FOG Setup
Fog is turned off by default in Carla (atlest on v0.9.13) for performance reasons.  To enable Fog, you need to modify the map of each town in Unreal Editor using the steps below:

- 1. Open Unreal Editor ("make launch" from Carla root directory will do this for you)
- 2. Open a town's map in Unreal Editor
- 3. Look for BP_Sky
- 4. Scroll down to "Volumetric Fog"
- 5. Check the box to the right of it
- 6. Save the map
- 7. Repeat steps 2-6 for all towns
- 8. Rebuild the package ("make package" from Carla root directory will do this for you)


HUD Changes
To disable the red reticle
- go to the WindowsNoEditor\CarlaUE\Config folder and open the file DReyeVRConfig.ini
- Change the line with DrawFlatReticle to the following:
   DrawFlatReticle=False; reticle in flat-screen mode
   
