# Release Information

## Version 10.1 (20240919-122750)

### Enhancements
* Adds new OpenCV-based `VisionProcessor`s (which may be attached to a VisionPortal in either Java or Blocks) to help teams implement color processing via computer vision in the INTO THE DEEP game
    * `ColorBlobLocatorProcessor` implements OpenCV color "blob" detection. A new sample program `ConceptVisionColorLocator` demonstrates its use.
        * A choice is offered between pre-defined color ranges, or creating a custom one in RGB, HSV, or YCrCb color space
        * The ability is provided to restrict detection to a specified Region of Interest on the screen
        * Functions for applying erosion / dilation morphing to the threshold mask are provided
        * Functions for sorting and filtering the returned data are provided
    * `PredominantColorProcessor` allows using a region of the camera as a "long range color sensor" to determine the predominant color of that region. A new sample program `ConceptVisionColorSensor` demonstrates its use.
        * The determined predominant color is selected from a discrete set of color "swatches", similar to the MINDSTORMS NXT color sensor
    * Documentation on this Color Processing feature can be found here: https://ftc-docs.firstinspires.org/color-processing
* Added Blocks sample programs for color sensors: RobotAutoDriveToLine and SensorColor.
* Updated Self-Inspect to identify mismatched RC/DS software versions as a "caution" rather than a "failure."

### Bug Fixes
* Fixes [AngularVelocity conversion regression](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070)

## Version 10.0  (20240828-111152)

### Breaking Changes
@@ -469,7 +487,7 @@ This is a bug fix only release to address the following four issues.
* Fixes [issue #260](https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/260) Blocks can't call java method that has a VuforiaLocalizer parameter.
    * Blocks now has a block labeled VuforiaFreightFrenzy.getVuforiaLocalizer for this.
* Added a page to manage the TensorFlow Lite models in /sdcard/FIRST/tflitemodels. To get to the TFLite Models page:
    * You can click on the link at the bottom of the the Manage page.
    * You can click on the link at the bottom of the Manage page.
    * You can click on the link at the upper-right the Blocks project page.
* Fixes logspam when `isBusy()` is called on a motor not in RTP mode.
* Hides the "RC Password" item on the inspection screen for phone-based Robot Controllers. (It is only applicable for Control Hubs).
  @@ -1154,7 +1172,7 @@ Known issues:

This version of the software provides support for the REV Robotics Expansion Hub.  This version also includes improvements in the USB communication layer in an effort to enhance system resiliency.  If you were using a 2.x version of the software previously, updating to version 3.1 requires that you also update your Driver Station software in addition to updating the Robot Controller software.

Also note that in version 3.10 software, the setMaxSpeed and getMaxSpeed methods are no longer available (not deprecated, they have been removed from the SDK). Also note that the the new 3.x software incorporates motor profiles that a user can select as he/she configures the robot.
Also note that in version 3.10 software, the setMaxSpeed and getMaxSpeed methods are no longer available (not deprecated, they have been removed from the SDK). Also note that the new 3.x software incorporates motor profiles that a user can select as he/she configures the robot.

Changes include:
* Blocks changes