const { useState, useEffect } = React;

function Login({ onLogin }) {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    const form = new URLSearchParams();
    form.append('username', username);
    form.append('password', password);
    const res = await fetch('http://localhost:8000/token', {
      method: 'POST',
      body: form,
    });
    if (res.ok) {
      const data = await res.json();
      onLogin(data.access_token);
    } else {
      alert('Login failed');
    }
  };

  return (
    <form onSubmit={handleSubmit}>
      <input value={username} onChange={e=>setUsername(e.target.value)} placeholder="user" />
      <input type="password" value={password} onChange={e=>setPassword(e.target.value)} placeholder="password" />
      <button type="submit">Login</button>
    </form>
  );
}

function Dashboard({ token }) {
  const [messages, setMessages] = useState([]);
  const [command, setCommand] = useState('');
  const [role, setRole] = useState('');

  useEffect(() => {
    const payload = atob(token).split(':');
    setRole(payload[1]);
    const ws = new WebSocket(`ws://localhost:8000/ws?token=${token}`);
    ws.onmessage = (e) => {
      setMessages((m) => [...m, e.data]);
      updateChart(e.data);
    };
    initMap();
    initChart();
    return () => ws.close();
  }, []);

  const sendCommand = async () => {
    await fetch('http://localhost:8000/publish', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify({ message: command }),
    });
    setCommand('');
  };

  return (
    <div>
      <div id="map" style={{ height: '200px' }}></div>
      <canvas id="chart" height="100"></canvas>
      {role === 'operator' && (
        <div>
          <input value={command} onChange={e=>setCommand(e.target.value)} placeholder="command" />
          <button onClick={sendCommand}>Send</button>
        </div>
      )}
      <ul>
        {messages.map((m, i) => <li key={i}>{m}</li>)}
      </ul>
    </div>
  );
}

function initMap() {
  const map = L.map('map').setView([0, 0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
}

let chart;
function initChart() {
  const ctx = document.getElementById('chart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line',
    data: { labels: [], datasets: [{ label: 'Sensor', data: [] }] },
  });
}
function updateChart(value) {
  if (!chart) return;
  chart.data.labels.push('');
  chart.data.datasets[0].data.push(Number(value));
  chart.update();
}

function App() {
  const [token, setToken] = useState(null);
  return token ? <Dashboard token={token} /> : <Login onLogin={setToken} />;
}

ReactDOM.render(<App />, document.getElementById('root'));
