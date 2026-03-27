# generated from rosidl_generator_py/resource/_idl.py.em
# with input from bunker_msgs:msg/BunkerRCState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BunkerRCState(type):
    """Metaclass of message 'BunkerRCState'."""

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
                'bunker_msgs.msg.BunkerRCState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bunker_rc_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bunker_rc_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bunker_rc_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bunker_rc_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bunker_rc_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BunkerRCState(metaclass=Metaclass_BunkerRCState):
    """Message class 'BunkerRCState'."""

    __slots__ = [
        '_swa',
        '_swb',
        '_swc',
        '_swd',
        '_stick_right_v',
        '_stick_right_h',
        '_stick_left_v',
        '_stick_left_h',
        '_var_a',
    ]

    _fields_and_field_types = {
        'swa': 'uint8',
        'swb': 'uint8',
        'swc': 'uint8',
        'swd': 'uint8',
        'stick_right_v': 'int8',
        'stick_right_h': 'int8',
        'stick_left_v': 'int8',
        'stick_left_h': 'int8',
        'var_a': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.swa = kwargs.get('swa', int())
        self.swb = kwargs.get('swb', int())
        self.swc = kwargs.get('swc', int())
        self.swd = kwargs.get('swd', int())
        self.stick_right_v = kwargs.get('stick_right_v', int())
        self.stick_right_h = kwargs.get('stick_right_h', int())
        self.stick_left_v = kwargs.get('stick_left_v', int())
        self.stick_left_h = kwargs.get('stick_left_h', int())
        self.var_a = kwargs.get('var_a', int())

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
        if self.swa != other.swa:
            return False
        if self.swb != other.swb:
            return False
        if self.swc != other.swc:
            return False
        if self.swd != other.swd:
            return False
        if self.stick_right_v != other.stick_right_v:
            return False
        if self.stick_right_h != other.stick_right_h:
            return False
        if self.stick_left_v != other.stick_left_v:
            return False
        if self.stick_left_h != other.stick_left_h:
            return False
        if self.var_a != other.var_a:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def swa(self):
        """Message field 'swa'."""
        return self._swa

    @swa.setter
    def swa(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'swa' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'swa' field must be an unsigned integer in [0, 255]"
        self._swa = value

    @property
    def swb(self):
        """Message field 'swb'."""
        return self._swb

    @swb.setter
    def swb(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'swb' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'swb' field must be an unsigned integer in [0, 255]"
        self._swb = value

    @property
    def swc(self):
        """Message field 'swc'."""
        return self._swc

    @swc.setter
    def swc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'swc' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'swc' field must be an unsigned integer in [0, 255]"
        self._swc = value

    @property
    def swd(self):
        """Message field 'swd'."""
        return self._swd

    @swd.setter
    def swd(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'swd' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'swd' field must be an unsigned integer in [0, 255]"
        self._swd = value

    @property
    def stick_right_v(self):
        """Message field 'stick_right_v'."""
        return self._stick_right_v

    @stick_right_v.setter
    def stick_right_v(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'stick_right_v' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'stick_right_v' field must be an integer in [-128, 127]"
        self._stick_right_v = value

    @property
    def stick_right_h(self):
        """Message field 'stick_right_h'."""
        return self._stick_right_h

    @stick_right_h.setter
    def stick_right_h(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'stick_right_h' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'stick_right_h' field must be an integer in [-128, 127]"
        self._stick_right_h = value

    @property
    def stick_left_v(self):
        """Message field 'stick_left_v'."""
        return self._stick_left_v

    @stick_left_v.setter
    def stick_left_v(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'stick_left_v' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'stick_left_v' field must be an integer in [-128, 127]"
        self._stick_left_v = value

    @property
    def stick_left_h(self):
        """Message field 'stick_left_h'."""
        return self._stick_left_h

    @stick_left_h.setter
    def stick_left_h(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'stick_left_h' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'stick_left_h' field must be an integer in [-128, 127]"
        self._stick_left_h = value

    @property
    def var_a(self):
        """Message field 'var_a'."""
        return self._var_a

    @var_a.setter
    def var_a(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'var_a' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'var_a' field must be an integer in [-128, 127]"
        self._var_a = value
