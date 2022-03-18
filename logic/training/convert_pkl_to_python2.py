import pickle

with open("trained_agent_saved_backup.pkl", "rb") as f:
    w = pickle.load(f)

pickle.dump(w, open("trained_agent_saved.pkl","wb"), protocol=2)