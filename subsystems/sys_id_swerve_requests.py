from typing import TYPE_CHECKING
from phoenix6.status_code import StatusCode

from phoenix6.swerve.requests import SwerveRequest
from phoenix6.swerve.swerve_module import SwerveModule
from phoenix6.controls.coast_out import CoastOut
from phoenix6.controls import PositionTorqueCurrentFOC, TorqueCurrentFOC

if TYPE_CHECKING:
    from phoenix6.swerve.swerve_drivetrain import SwerveControlParameters

class SysIdSwerveTranslation(SwerveRequest):
    """
    SysId-specific SwerveRequest to characterize the translational
    characteristics of a swerve drivetrain.
    """

    def __init__(self):
        self.amps_to_apply = 0.0

        self.__drive_request = TorqueCurrentFOC(0)
        self.__steer_request = PositionTorqueCurrentFOC(0)

    def apply(self, parameters: 'SwerveControlParameters', modules_to_apply: list[SwerveModule]) -> StatusCode:
        for module in modules_to_apply:
            module.apply(
                self.__drive_request.with_output(self.amps_to_apply),
                self.__steer_request.with_position(0)
            )
        return StatusCode.OK

    def with_amps(self, amps) -> 'SysIdSwerveTranslation':
        """
        Sets the amperage to apply to the drive wheels.

        :param amps: Amperage to apply
        :type amps: ampere
        :returns: this request
        :rtype: SysIdSwerveTranslation
        """
        self.amps_to_apply = amps
        return self
    
class SysIdSwerveSteerGains(SwerveRequest):
    """
    SysId-specific SwerveRequest to characterize the steer module
    characteristics of a swerve drivetrain.
    """

    def __init__(self):
        self.amps_to_apply = 0.0

        self.__drive_request = CoastOut()
        self.__steer_request = TorqueCurrentFOC(0)

    def apply(self, parameters: 'SwerveControlParameters', modules_to_apply: list[SwerveModule]) -> StatusCode:
        for module in modules_to_apply:
            module.apply(
                self.__drive_request,
                self.__steer_request.with_output(self.amps_to_apply)
            )
        return StatusCode.OK

    def with_amps(self, amps) -> 'SysIdSwerveSteerGains':
        """
        Sets the amperage to apply to the steer wheels.

        :param amps: Amperage to apply
        :type amps: ampere
        :returns: this request
        :rtype: SysIdSwerveSteerGains
        """
        self.amps_to_apply = amps
        return self
