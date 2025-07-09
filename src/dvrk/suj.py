#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    class __Arm:

        # local kinematics
        class __Local:
            def __init__(self, ral, connection_timeout):
                self.__crtk_utils = crtk.utils(self, ral, connection_timeout)
                self.__crtk_utils.add_measured_cp()

        def __init__(self, ral, connection_timeout):
            self.__ral = ral
            self.__crtk_utils = crtk.utils(self, ral, connection_timeout)
            self.__crtk_utils.add_operating_state()
            self.__crtk_utils.add_measured_js()
            self.__crtk_utils.add_measured_cp()
            self.__crtk_utils.add_move_jp() # for simulated SUJs only
            self.local = self.__Local(ral.create_child('/local'), connection_timeout)

        def ral(self):
            return self.__ral

    # initialize the all SUJ arms
    def __init__(self, ral, connection_timeout = 1.0):
        """Constructor.  This initializes a few data members and creates
        instances of classes for each SUJ arm."""
        self.__ral = ral.create_child('SUJ')
        self.__crtk_utils = crtk.utils(self, ral, connection_timeout)
        for arm in ('ECM', 'PSM1', 'PSM2', 'PSM3'):
            setattr(self, arm, self.__Arm(ral.create_child(arm), connection_timeout))

    def ral(self):
        return self.__ral
