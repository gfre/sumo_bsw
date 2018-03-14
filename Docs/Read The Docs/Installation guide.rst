.. _Installation Guide:

=========================================
Installation guide
=========================================
This documentation guides you through the installation of needed software to get you 
up and running with developing new basis software (BSW) for the ACON Sumo robot.

.. hint:: 	If you run into any problems, these blog entries from Erich Styger might help:
				1. `Importing Kinetis Design Studio Projects <https://mcuoneclipse.com/2017/04/02/mcuxpresso-ide-importing-kinetis-design-studio-projects/>`_
				2. `Installing Processor Expert <https://mcuoneclipse.com/2017/04/09/mcuxpresso-ide-installing-processor-expert-into-eclipse-neon/>`_
				3. `MCUXpresso IDE 10.1 <https://mcuoneclipse.com/2017/11/25/eclipse-mcuxpresso-ide-10-1-with-integrated-mcuxpresso-configuration-tools/>`_
				
			Also, take a look at the `GNU ARM/MCU Eclipse documentation <https://gnu-mcu-eclipse.github.io/>`_
			
-----------------
Required accounts
-----------------
I. GitHub account
	This gets you access to the Sumo BSW repository containing all the source files
II. NXP account
	This is needed to download and install the MCUXpresso IDE
	
--------
Software
--------
:ref:`install MCUXPresso`
	This is the eclipse-based integrated development environment (IDE) in which you write your code,
	flash the Sumo, debug, etc...
:ref:`installSmartGit` (not mandatory but recommended)
	This will help you manage your file history in the process of making new software. 
	Skip this if you already use some sort of git version control.
:ref:`install FreeMaster`
	This is a run-time debugging tool that lets you plot data over a USB- or Bluetooth-connection to the Sumo.
	
	.. note:: If you want to use FreeMaster, then you **must** use a Windows platform, since FreeMaster 
			  is only available for Windows.


.. _install MCUXpresso:

I. Installing the MCUXpresso IDE
--------------------------------
Since the Sumo robot is a somewhat proprietary piece of hardware developed by Erich Styger at the Lucerne School
Of Engineering And Architecture, the installation process of the MCUXpresso IDE will include parts from his blog
`MCU on eclipse`_. 

Go to the `MCUXpresso download page <https://www.nxp.com/support/developer-resources/software-development-tools/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE>`_
and download the newest version available. You need to have an NXP-account to do that. Afterwards, just follow the installation steps
and leave everything as-is. 

.. note:: This should also install the *Segger J-LinkSoftware And Documentation Pack*, which you 
		  will need to debug and communicate with the Sumo via the *RTT-Viewer* tool. If the 
		  installation for some reason failed, you can download the package 
		  `here <https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack>`_ .

The next step is needed because the Sumo uses Processor Expert (PE), a software tool to generate (low-level)
code on the fly. This is not generically supported by the MCUXpresso IDE (as it originates from the Kinetis 
Design Studio IDE) but you can still import projects that use this feature. However, you can't (yet) configure/produce new
files with Processor Expert in MCUXpresso. PE, in turn, needs *GNU tools* for compilation and building.

(i). GNU ARM Toolchain
**********************
Follow `this link <https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads>`_ and download 
the file *gcc-arm-none-eabi-6-2017-q1-update-win32.zip* from February 23, 2017.

.. note:: You can try to download the newest version of the *GNU Arm Embedded Toolchain*, but this has not been tested.

Open the archive and extract the *gcc-arm-none-eabi-6-2017-q1-update-win32* folder to 
*~/Appdata/Roaming/GNU ARM Embedded* 

.. note:: By default, the *Appdata* directory is not visible. You can get to the directory by typing \"%APPDATA%\" 
			into an Explorer window and pressing enter
			
				.. figure:: images/appdata.png
	
				Getting to the *Appdata* directory

.. hint:: To check if the *GNU ARM gcc compiler* is functional, open a *Terminal* window, change directory to *~/AppData/Roaming/GNU ARM Embedded/
			gcc-arm-none-eabi-6-2017-q1-update-win32/bin* (by using the command \"cd\") and type \"arm-none-eabi-gcc --version\". 
			
				.. figure:: images/gcctest.png

				Checking if the *ARM C compiler* is functional.

(ii). Windows Build Tools
*************************
.. note:: This step is only necessary if you use a Windows machine.

These tools seem necessary, since Windows does not provide a *make* program by
default. The installation process is similar to the one before. Visit `this website <https://github.com/gnu-mcu-eclipse/windows-build-tools/releases>`_
and download the *gnu-mcu-eclipse-build-tools-x.xx-xxxxxxxx-xx-win64.zip* file. Extract the *GNU MCU Eclipse* folder
to the path *~/Appdata/Roaming/GNU MCU Eclipse*.

.. hint:: To check if the *GNU MCU Eclipse Windows Build Tools* are functional, open a *Terminal* window, change directory to *~/AppData/
			Roaming/GNU MCU Eclipse/Build Tools/2.10-20180103-1919/bin* (by using the command \"cd\") and type 
			\"make --version\". 
			
				.. figure:: images/maketest.png

				Checking if the *GNU MCU Eclipse Windows Build Tools* are functional.
				
(iii). GNU MCU Eclipse plugin
*****************************
To have Eclipse working with these tools, you need to install GNU MCU Eclipse plugin in the MCUXpresso IDE. 
On the `GitHub Wiki <https://gnu-mcu-eclipse.github.io/plugins/install/>`_ of the GNU MCU Eclipse tools it says that \"*Starting with v4.x, 
the oldest Eclipse supported by the plug-ins is Eclipse 4.6 with C Developing Tools (CDT) 9.2*\". I am installing with MCUXpresso 10.1.1 Build 606 2018-01-02,
Eclipse	4.6.3v20170301-0400 (Neon)	with CDT 9.1.0.201609121658. The newest GNU MCU Eclipse version would therefore not install correctly. Erich Styger
recommends installing the `GNU ARM/MCU Eclipse plug-ins version 3.4.1  <https://github.com/gnu-mcu-eclipse/eclipse-plugins/releases/tag/v3.4.1-201704251808>`_ , which 
worked for me. If you have a different MCUXpresso version, you should check compatibility on the GitHub Wiki.

If you downloaded the *.zip*-file, open *MCUXpresso->Help->Install New software*, and put the file into the upcoming window via *drag\&drop*. It should 
now show up as shown in the picture. Mark the box and finish the installation.

.. figure:: images/installnewsoftware.png

	How to install new software in MCUXpresso.


.. figure:: images/installgnumcueclipse.png

	*Drag \& drop* the *.zip*-file to the window, check the box and finish the installation.
				
				
(iv). Setting up MCUXpresso to use the ARM toolchain
****************************************************
In this step, we need to make sure, that MCUXpresso *knows* where to look for the *ARM Toolchain* and the *Windows Build Tools* (if you use Windows). This can
be done in the Preferences. Go to *Window->Preferences*, expand *C/C++*, expand *Build* and go to *Workspace Tools Paths*. As shown in the picture, the Toolchain should be
set to the *GNU Tools for ARM Embedded Processors*. You may need to copy the path to the *Windows Build Tools* and the *GNU ARM Toolchain* manually. Click *Apply* and *OK*.

.. figure:: images/checktools.png
	
	Tool chain preferences for the C build. You may need to copy the paths manually into the boxes.

	
.. note:: This way you set the default Toolchain for all projects in the workspace. If you dont want that, you can set them for each project individually in the
			project settings.
				
	
(v). Setting up MCUXpresso to use Processor Expert
**************************************************
If you installed MCUXpresso v10.1 or newer, you need to first deinstall the *MCUCpresso IDE Configuration Tools*. These are the first step to replace 
PE, but you can still use it separately. In order to do so and to avoid conflicts between the *Configuration Tools* and PE, you need to deinstall them.
Go to *Help->Installation Details* and search for *MCUXpresso IDE Configuration Tools Integration*, select it, and click *Uninstall*.

.. figure:: images/uninstallconfigurationtools.png
	
	Uninstall the *Configuration Tools* to avoid conflicts with PE.
	
Now, go to the `Processor Expert Download Page <https://www.nxp.com/pages/processor-expert-software-microcontroller-driver-suite:PE_DRIVER_SUITE?&&tab=Design_Tools_Tab>`_ 
and download the *Processor Expert for Kinetis v3.0 Eclipse plugin* 

.. figure:: images/processorexpertplugin.png

	Download the PE plugin *.zip*-file.
	
After unzipping this file, install the *freescale_updater.zip*-file via *Help->Install New Software* as described above.

.. attention:: You must install the *freescale updater*-file first! 

.. figure:: images/freescaleupdaterfirst.png

	Install the marked file from the *Processor Expert for Kinetis v3.0 Eclipse plugin*-file first. Then install *PEx for Kinetis v3.0.0*.

Afterwards, install *PEx for Kinetis 3.0.0* in the same way. Then, download, unzip, and install the file *Processor Expert for Kinetis v3.0.2 update*
in the same way.

.. figure:: images/processorexpertupdate.png
	
	Install the update *PEx for Kinetis 3.0.2*
	
In the final step you need to download the PE components from `Erich Stygers Sourceforge repo <https://sourceforge.net/projects/mcuoneclipse/files/PEx%20Components/>`_ 
You should check with which version the Sumo robot is currently working, download the fitting release (currently *Components 2017-12-26.zip* (14.03.2018)). Unzip
the file, go to *Processor Expert->Import Components* search and select the *.PEupd*-files and click *import*.

.. figure:: images/processorexpertimportcomponents.png
	
	Install the PE components from Erich Stygers Sourceforge repo.

.. _importsumoproject:

(vi). Import the Sumo source code and verify settings
*****************************************************
To verify that everything is set up correctly, import the Sumo project source code. If you installed SmartGit and did the steps according to :ref:`installSmartGit` ,
you can open the project in MCUXPresso by selecting *File->Open Projects from File System* as seen in the figure below

.. figure:: images/openproject.png
	
	How to open the project in MCUXpresso.

If you dont see a *Generated Code* folder in the project, go to *Processor Expert->Show Views* and click *Generate Processor Expert Code* as shown below.

.. figure:: images/generatecode.png

	If you **DON'T** see the *Generated Code* folder, hit *Generate Processor Expert Code*.

This process may take a while and generates all the low-level code for the required components of the Sumo project. When finished click *Edit \'sumo_bsw project settings\'*
(you need to have the project selected to do this) and expand *C/C++ build*. The settings of *Settings, Tool Chain Editor,* and *Tool Paths* should show as in the pictures below
	
.. figure:: images/projectsettings1.png

	The target processor should be *cortex-m4*. If it isn't go to the *MCU settings* and select *Generic-M4*.
	
.. figure:: images/projectsettings2.png

	Because we use the ARM toolchain, *Cross ARM GCC* should be selected.
	
.. figure:: images/projectsettings3.png
	
	The *Tool Path* should be set to *GNU Tools for ARM Embedded Processors*.
	
	
.. _includedoxygen:

(vii). Include doxygen tools to MCUXpresso
******************************************
Doxygen is used to write code documentation for the project. To integrate it to MCUXpresso, go to the *Eclipse Marketplace* and search and install
*Eclox*. 

.. figure:: images/eclox.png

	Eclox integrates doxygen support to the MCUXPresso IDE.
	
Since doxygen uses the *dot* language and *GraphViz*, you also need to `download Graphviz <https://graphviz.gitlab.io/_pages/Download/Download_windows.html>`_
,e.g., *graphviz-2.38.zip*. Unpack the content of the *release*-folder to \"*~/Program Files(x86)/Graphviz2.38*\". Now make sure that the *sumo_bsw.doxyfile*, which is
located in the folder *Docs* in the *sumo_bsw project*, is set up correctly by double-clicking it. 

.. note:: If you haven't imported the Sumo_BSW project repository to your workspace yet, see :ref:`installSmartGit` and :ref:`importSumoProject`.

Go to the *Advanced* tab and search for *DOT Path*. This must contain the
path to the *dot.exe* in the *Graphviz* directory as shown in the picture.

.. figure:: images/doxyfilepreferences.png

	The *DOT Path* must be correct in order to get doxygen working correctly.

To try if Eclox works, you can generate the documentation as a test. To do this, click the blue \@-symbol in the toolbar or, alternatively, right-click the
*sumo_bsw.doxyfile* in *sumo_bsw->Docs* and click *Build documentation* as described in the picture below.

.. figure:: images/generatedocs.png

	There are two ways to generate the documentation files: hit the \@-symbol or right-click the doxyfile and *Build documentation*.

This should generate several files in the *html*-folder. Double-clicking the *index.html*-file opens the documentation. Search for an arbitrary file and if
you see something like in the picture below, everything works fine (given you don't have an error output).

.. figure:: images/documentationexample.png

	When Eclox works, you should see that there is a graph showing dependencies of any arbitrary file.
	
.. hint:: If Eclox/doxygen still generates an error like \"*error: problems running dot exit code=-1*\" then try to set the PATH variable, so Eclox can
			find the *dot.exe* there. You can do that by *right-clicking the Windows symbol->System->System info->Advanced System Settings->Environment Variables->PATH*
			and adding the path to the *bin*-folder. Also, restart MCUXpresso.
				
				.. image:: images/pathvariable.png

				

.. _installSmartGit:

II. Installing SmartGit
-----------------------

.. note:: If you use other version control software you can skip this. In that case, all you need is the `URL to the Sumo BSW`_ and clone
			the entire repo to *~/Documents/MCUXpressoIDE_xx.x.x_xxx/workspace/sumo_bsw* and the *gh-pages*-branch to *~/Documents/MCUXpressoIDE_xx.x.x_xxx/workspace/sumo_bsw/Docs/html*.

Follow this `link <https://www.syntevo.com/smartgit/>`_ and download SmartGit. Install SmartGit for *non-commercial use only*. That
way you have all features enabled.
	
.. figure:: images/smartgit1.png
	
	Installing SmartGit under the *non-commercial license*.

Afterwards, select SmartGit as your SSH-Client and GitHub as your hosting provider. For this, you need to generate an *API Token*
that identifies you with your GitHub account. 

.. figure:: images/smartgit2.png
	
	Selecting GitHub as your hosting provider and verifying your account.
	
After clicking *Generate API Token* you will be redirected to a website with your 
API token on it. Copy this token in the Token field and go on. SmartGit will then ask you what to do next. Hit *clone existing repository*
and enter the `URL to the Sumo BSW`_ . You may have to enter your GitHub password for this.
Finally, select the location on your disk where the Repo will be cloned into.
I recommend to clone it to *~/Documents/MCUXpressoIDE_xx.x.x_xxx/workspace/sumo_bsw*.
You should now have all the source files in that directory.

To document the code using doxygen (see :ref:`includedoxygen`), we use the *gh-pages* branch from the Sumo repo to update the doxygen files.
Therefore we define a new, local repo *html* that we place inside the *Sumo BSW repo*\'s *Doc*-folder. To achieve this, check out to the 
master or development branch of the Sumo repo in SmartGit. Then, create a new folder in the *sumo_bsw/Docs/*-directory. In SmartGit, select
*Repository->Clone* and similar to above, select the GitHub Website for the sumo_bsw repo. But now, select the *gh-pages*-branch as shown below.

.. figure:: images/smartgit_githubpages.png

	Check out to the *gh-pages*-branch when creating the new *html*-repo.
	
Now select the *html*-folder that you just created and you are done!

.. _install FreeMaster:

III. Installing FreeMaster
--------------------------
Visit the `FreeMaster download page <https://www.nxp.com/support/developer-resources/software-development-tools/freemaster-run-time-debugging-tool:FREEMASTER?tab=Design_Tools_Tab>`_
and download the *FreeMASTER 2.0 Application Installation*-file. Execute the file and select *complete installation*.

.. figure:: images/freemasterinstall.png

	Download and install the FreeMaster.

.. URLs
.. _`MCU on eclipse`: https://mcuoneclipse.com/
.. _`FreeMaster`: https://www.nxp.com/support/developer-resources/software-development-tools/freemaster-run-time-debugging-tool:FREEMASTER
.. _`MCUXpresso`: https://www.nxp.com/support/developer-resources/software-development-tools/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE
.. _`SmartGit`: https://www.syntevo.com/smartgit
.. _`URL to the Sumo BSW`: https://github.com/gfre/sumo_bsw