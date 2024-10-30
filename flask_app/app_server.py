import socket
from flask import Flask, request, jsonify, render_template, Response, redirect, url_for
import subprocess
app=Flask(__name__) #instance flask class

sos_mess=[]
@app.route('/')
def show_home():
    return "Welcome to the Home Page!"

@app.route('/receive_sos', methods=['POST'])
def receive():
    try:
        sos_data=request.get_json()
        
        if 'message' not in sos_data :
            print("Error in fetching sos data")
        mess=sos_data['message']
        if not mess.strip():
            print("SOS can't be empty")
        sos_mess.append(mess)
        print(f"Receive SOS!!!: {mess}")
        return redirect(url_for('home'))
    
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
    
@app.route('/store_sos', methods=['GET'])
def get_sos():
    return jsonify({'messages': sos_mess})
    
@app.route('/logs')
def show_logs():
    return render_template('Logs.html', sos_messages=sos_mess)

@app.route('/about')
def about():
    return render_template('about.html')
@app.route('/home')
def home():
    sos_count = len(sos_mess)
    return render_template('home.html', sos_count=sos_count)





@app.route('/deployed')
def deploy():
    return render_template('deployed.html')


@app.route('/start-simulation', methods=['POST'])
def start_simulation():
    try:
        # Command to launch the Webots simulation
        #Replace with your path for webots and world file

        subprocess.Popen(['/Applications/Webots.app/Contents/MacOS/webots', '--mode=fast', '/Users/seema/Desktop/flask_app/mavic/worlds/forest.wbt'])

        return jsonify({"message": "Webots simulation has started successfully!"})
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500



if __name__=="__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)

