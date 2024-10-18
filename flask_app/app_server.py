from flask import Flask, request, jsonify, render_template
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
        return jsonify({'status':'SOS received', 'message': mess}), 200
    
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
    return render_template('home.html')
@app.route('/deployed')
def deploy():
    return render_template('deployed.html')

if __name__=="__main__":
    app.run(host='0.0.0.0', port=5001, debug=True)

