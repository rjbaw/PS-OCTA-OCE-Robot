Howdy Ren/Vattanary, hope things are doing well

Here's all the leftover CAD files I made during my time here, lots of the off-the-shelf components I got off Thorlabs, they have a very comprehensive library of CAD files for all their products, so I didn't include all of those.

I'll try to add stuff I've missed in the future, and you should feel free to email me with any questions, but here's a summary of what I've got. Everything that can be is saved as a Fusion360 file, as well as a STEP file. STEP files give you less information about the part history, but will be more compatible with other CAD softwares. The 3D printed parts have their latest versions as a 3MF file in the relevant folder.

 - Chassis
	This is the aluminum body of the OCE scanner, I have it here for reference and for future 3D printing needs, but it is machined out of aluminum so unless you learn some CNC machining this is our final draft.
	Future projects you will need this for: one day Vanya wants there to be one solid, fixed bracing to slot the transducer in that's always accurate and has the transducer pointed at just the right spot. However, we keep needing to adjust where the transducer goes, so for now we just use an adjustable one.
	In the future you will need to design a fixed one once we have found where we want it to finally go. You may need to do this multiple times, so I would encourage studying some parametric-modeling techniques so you can iterate quickly.

 - Casing_final_hopefully
	This is the plastic casing that goes around the OCE scanner. As of 4 PM on June 28, I am printing it in Foege hall and it should hopefully be the last version we need, but in case the print fails or we need to modify it, a F360 file and 3MF have both been provided.

 - Galvo chassis (aka 'Chassis v1.step')
	This is a CAD file of the metal chassis of the galvo in the OCE head. Beware, this model does not include the motors, so you should keep that in mind if you use it for layout reference. It's important because the 3 M3 holes on the bottom define how the galvo interfaces with the chassis, but since we've already machined that chassis it should hopefully not be relevant.

 - Transducer_holder_again v18 (aka ceramic_holder_v7)
	This is the CAD file for the plastic transducer holder that we've been printing a lot of recently. There have been a lot of iterations on this one and we go through them quickly. Luckily it's a cheap and quick part to make, but any future transducers we need to make will need another one of these. The 3D printing file for this one is named 'ceramic_holder_v7,' which is the latest version.

 - UR3e_mount_mark3 v40
	This STEP file contains the full assembly of everything as it currently exists on the end of the galvo. For some reason I was unable to export this in Fusion360's native format so hopefully the STEP file suffices. It is useful as reference because the first estimation of the TCP offset we got by measuring a distance in the CAD file. Again, hopefully this does not need to be touched again, but it is still important you have access to it just in case.