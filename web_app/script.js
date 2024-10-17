document.getElementById('setGoal').addEventListener('click', async () => {
    const x = document.getElementById('x').value;
    const y = document.getElementById('y').value;
    const theta = document.getElementById('theta').value;

    try {
        const response = await fetch('http://localhost:8000/set_goal', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ x, y, theta }),
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        const result = await response.json();
        console.log(result);
        alert(result.message);
    } catch (error) {
        console.error('Error:', error);
        alert('Failed to set goal. Check the console for more details.');
    }
});

document.getElementById('startNav').addEventListener('click', async () => {
    try {
        const response = await fetch('http://localhost:8000/start_navigation', {
            method: 'POST',
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        const result = await response.json();
        console.log(result);
        alert(result.message);
    } catch (error) {
        console.error('Error:', error);
        alert('Failed to start navigation. Check the console for more details.');
    }
});