db = canDatabase("can_signals.dbc")
db.Messages
msgControlInfo = canMessage(db, 'ControlMessage')


msgControlInfo.Signals
msgEngineInfo.Signals.Rotate_RL = 5000
msgControlInfo.Signals
