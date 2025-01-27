
# static final double MAX_VELOCITY // m/s
|-------|---------------------------        |
| 1.0   |   still aggressive                |
| 0.5   |   is more conservative            |
| 0.25  |   Seems better, also changed P to 0.25 |
| 0.15  |


Changing seems to have no impact

# static final double MAX_ACCELERATION // m/s²
|-------|---------------------------    |
| 1.5   |   still aggressive
| 0.5   |   is more conservative
Changing seems to have no impact

# static final double MAX_ROTATION_VELOCITY // rad/s
|   2.0 |
|   1.0 |


# static final double MAX_ROTATION_ACCELERATION // rad/s²
|   3.0 |

#  Tolerances
## static final double POSITION_TOLERANCE // meters
|   0.02    |
|   0.10    | Seems better, also changed P to 0.25 |
|   0.15    |

##   static final double ROTATION_TOLERANCE // radians
|   0.02    |
|   0.02    |
|   0.04    |

# // PID Gains
##        static final double TRANSLATION_P
|   0.5     |
|   0.25    | Seemed less aggressive
|   0.10    | Better than 0.25
|   0.05    | Erratic
|   0.08    |

##        static final double TRANSLATION_I
|   0.0     |

##        static final double TRANSLATION_D
|   0.0     |
|   0.01    |

##        static final double ROTATION_P
|   1.0     |

##        static final double ROTATION_I
|   0.0     |

##        static final double ROTATION_D
|   0.1     |
|   0.05    |

When approaching target the wheels act erratically 