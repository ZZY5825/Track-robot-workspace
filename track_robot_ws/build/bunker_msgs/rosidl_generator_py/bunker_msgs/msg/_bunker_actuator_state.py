# generated from rosidl_generator_py/resource/_idl.py.em
# with input from bunker_msgs:msg/BunkerActuatorState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BunkerActuatorState(type):
    """Metaclass of message 'BunkerActuatorState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'bunker_msgs.msg.BunkerActuatorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bunker_actuator_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bunker_actuator_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bunker_actuator_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bunker_actuator_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bunker_actuator_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BunkerActuatorState(metaclass=Metaclass_BunkerActuatorState):
    """Message class 'BunkerActuatorState'."""

    __slots__ = [
        '_motor_id',
        '_rpm',
        '_current',
        '_pulse_count',
        '_driver_voltage',
        '_driver_temperature',
        '_motor_temperature',
        '_driver_state',
    ]

    _fields_and_field_types = {
        'motor_id': 'uint8',
        'rpm': 'int16',
        'current': 'double',
        'pulse_count': 'int32',
        'driver_voltage': 'float',
        'driver_temperature': 'float',
        'motor_temperature': 'int8',
        'driver_state': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.motor_id = kwargs.get('motor_id', int())
        self.rpm = kwargs.get('rpm', int())
        self.current = kwargs.get('current', float())
        self.pulse_count = kwargs.get('pulse_count', int())
        self.driver_voltage = kwargs.get('driver_voltage', float())
        self.driver_temperature = kwargs.get('driver_temperature', float())
        self.motor_temperature = kwargs.get('motor_temperature', int())
        self.driver_state = kwargs.get('driver_state', int())

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
        if self.motor_id != other.motor_id:
            return False
        if self.rpm != other.rpm:
            return False
        if self.current != other.current:
            return False
        if self.pulse_count != other.pulse_count:
            return False
        if self.driver_voltage != other.driver_voltage:
            return False
        if self.driver_temperature != other.driver_temperature:
            return False
        if self.motor_temperature != other.motor_temperature:
            return False
        if self.driver_state != other.driver_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def motor_id(self):
        """Message field 'motor_id'."""
        return self._motor_id

    @motor_id.setter
    def motor_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor_id' field must be an unsigned integer in [0, 255]"
        self._motor_id = value

    @property
    def rpm(self):
        """Message field 'rpm'."""
        return self._rpm

    @rpm.setter
    def rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rpm' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'rpm' field must be an integer in [-32768, 32767]"
        self._rpm = value

    @property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
        self._current = value

    @property
    def pulse_count(self):
        """Message field 'pulse_count'."""
        return self._pulse_count

    @pulse_count.setter
    def pulse_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pulse_count' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'pulse_count' field must be an integer in [-2147483648, 2147483647]"
        self._pulse_count = value

    @property
    def driver_voltage(self):
        """Message field 'driver_voltage'."""
        return self._driver_voltage

    @driver_voltage.setter
    def driver_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'driver_voltage' field must be of type 'float'"
        self._driver_voltage = value

    @property
    def driver_temperature(self):
        """Message field 'driver_temperature'."""
        return self._driver_temperature

    @driver_temperature.setter
    def driver_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'driver_temperature' field must be of type 'float'"
        self._driver_temperature = value

    @property
    def motor_temperature(self):
        """Message field 'motor_temperature'."""
        return self._motor_temperature

    @motor_temperature.setter
    def motor_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_temperature' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'motor_temperature' field must be an integer in [-128, 127]"
        self._motor_temperature = value

    @property
    def driver_state(self):
        """Message field 'driver_state'."""
        return self._driver_state

    @driver_state.setter
    def driver_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'driver_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'driver_state' field must be an unsigned integer in [0, 255]"
        self._driver_state = value
