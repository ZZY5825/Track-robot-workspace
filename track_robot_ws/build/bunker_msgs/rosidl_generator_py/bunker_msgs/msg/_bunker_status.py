# generated from rosidl_generator_py/resource/_idl.py.em
# with input from bunker_msgs:msg/BunkerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BunkerStatus(type):
    """Metaclass of message 'BunkerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MOTOR_ID_FRONT_RIGHT': 0,
        'MOTOR_ID_FRONT_LEFT': 1,
        'MOTOR_ID_REAR_RIGHT': 2,
        'MOTOR_ID_REAR_LEFT': 3,
        'LIGHT_ID_FRONT': 0,
        'LIGHT_ID_REAR': 1,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('bunker_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'bunker_msgs.msg.BunkerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bunker_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bunker_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bunker_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bunker_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bunker_status

            from bunker_msgs.msg import BunkerActuatorState
            if BunkerActuatorState.__class__._TYPE_SUPPORT is None:
                BunkerActuatorState.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MOTOR_ID_FRONT_RIGHT': cls.__constants['MOTOR_ID_FRONT_RIGHT'],
            'MOTOR_ID_FRONT_LEFT': cls.__constants['MOTOR_ID_FRONT_LEFT'],
            'MOTOR_ID_REAR_RIGHT': cls.__constants['MOTOR_ID_REAR_RIGHT'],
            'MOTOR_ID_REAR_LEFT': cls.__constants['MOTOR_ID_REAR_LEFT'],
            'LIGHT_ID_FRONT': cls.__constants['LIGHT_ID_FRONT'],
            'LIGHT_ID_REAR': cls.__constants['LIGHT_ID_REAR'],
        }

    @property
    def MOTOR_ID_FRONT_RIGHT(self):
        """Message constant 'MOTOR_ID_FRONT_RIGHT'."""
        return Metaclass_BunkerStatus.__constants['MOTOR_ID_FRONT_RIGHT']

    @property
    def MOTOR_ID_FRONT_LEFT(self):
        """Message constant 'MOTOR_ID_FRONT_LEFT'."""
        return Metaclass_BunkerStatus.__constants['MOTOR_ID_FRONT_LEFT']

    @property
    def MOTOR_ID_REAR_RIGHT(self):
        """Message constant 'MOTOR_ID_REAR_RIGHT'."""
        return Metaclass_BunkerStatus.__constants['MOTOR_ID_REAR_RIGHT']

    @property
    def MOTOR_ID_REAR_LEFT(self):
        """Message constant 'MOTOR_ID_REAR_LEFT'."""
        return Metaclass_BunkerStatus.__constants['MOTOR_ID_REAR_LEFT']

    @property
    def LIGHT_ID_FRONT(self):
        """Message constant 'LIGHT_ID_FRONT'."""
        return Metaclass_BunkerStatus.__constants['LIGHT_ID_FRONT']

    @property
    def LIGHT_ID_REAR(self):
        """Message constant 'LIGHT_ID_REAR'."""
        return Metaclass_BunkerStatus.__constants['LIGHT_ID_REAR']


class BunkerStatus(metaclass=Metaclass_BunkerStatus):
    """
    Message class 'BunkerStatus'.

    Constants:
      MOTOR_ID_FRONT_RIGHT
      MOTOR_ID_FRONT_LEFT
      MOTOR_ID_REAR_RIGHT
      MOTOR_ID_REAR_LEFT
      LIGHT_ID_FRONT
      LIGHT_ID_REAR
    """

    __slots__ = [
        '_header',
        '_linear_velocity',
        '_angular_velocity',
        '_vehicle_state',
        '_control_mode',
        '_error_code',
        '_battery_voltage',
        '_actuator_states',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'linear_velocity': 'double',
        'angular_velocity': 'double',
        'vehicle_state': 'uint8',
        'control_mode': 'uint8',
        'error_code': 'uint16',
        'battery_voltage': 'double',
        'actuator_states': 'bunker_msgs/BunkerActuatorState[3]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['bunker_msgs', 'msg'], 'BunkerActuatorState'), 3),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.linear_velocity = kwargs.get('linear_velocity', float())
        self.angular_velocity = kwargs.get('angular_velocity', float())
        self.vehicle_state = kwargs.get('vehicle_state', int())
        self.control_mode = kwargs.get('control_mode', int())
        self.error_code = kwargs.get('error_code', int())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        from bunker_msgs.msg import BunkerActuatorState
        self.actuator_states = kwargs.get(
            'actuator_states',
            [BunkerActuatorState() for x in range(3)]
        )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.linear_velocity != other.linear_velocity:
            return False
        if self.angular_velocity != other.angular_velocity:
            return False
        if self.vehicle_state != other.vehicle_state:
            return False
        if self.control_mode != other.control_mode:
            return False
        if self.error_code != other.error_code:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.actuator_states != other.actuator_states:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def linear_velocity(self):
        """Message field 'linear_velocity'."""
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'linear_velocity' field must be of type 'float'"
        self._linear_velocity = value

    @property
    def angular_velocity(self):
        """Message field 'angular_velocity'."""
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_velocity' field must be of type 'float'"
        self._angular_velocity = value

    @property
    def vehicle_state(self):
        """Message field 'vehicle_state'."""
        return self._vehicle_state

    @vehicle_state.setter
    def vehicle_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vehicle_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'vehicle_state' field must be an unsigned integer in [0, 255]"
        self._vehicle_state = value

    @property
    def control_mode(self):
        """Message field 'control_mode'."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'control_mode' field must be an unsigned integer in [0, 255]"
        self._control_mode = value

    @property
    def error_code(self):
        """Message field 'error_code'."""
        return self._error_code

    @error_code.setter
    def error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_code' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'error_code' field must be an unsigned integer in [0, 65535]"
        self._error_code = value

    @property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
        self._battery_voltage = value

    @property
    def actuator_states(self):
        """Message field 'actuator_states'."""
        return self._actuator_states

    @actuator_states.setter
    def actuator_states(self, value):
        if __debug__:
            from bunker_msgs.msg import BunkerActuatorState
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, BunkerActuatorState) for v in value) and
                 True), \
                "The 'actuator_states' field must be a set or sequence with length 3 and each value of type 'BunkerActuatorState'"
        self._actuator_states = value
