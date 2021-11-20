using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class RotacionMano : MonoBehaviour
{
	public SerialPort serialPort = new SerialPort ("COM4", 9600);
    // Start is called before the first frame update
	public float moveSpeed =50f;
	public float turnSpeed =50f;

    // Update is called once per frame
    void Start(){
		serialPort.Open ();
		serialPort.ReadTimeout = 100;
	}
	void Update(){

		try{
			
			if(serialPort.IsOpen){
				print(serialPort.ReadLine());
				Movimiento (int.Parse(serialPort.ReadLine()));
			}
		} catch (System.Exception ex){
			ex=new System.Exception();
		}

		if (Input.GetKey(KeyCode.Space))
			transform.Rotate(Vector3.forward *turnSpeed * Time.deltaTime);
		
		if (Input.GetKey(KeyCode.Escape))
			transform.Rotate(-Vector3.forward *turnSpeed * Time.deltaTime);
				
		
		if (Input.GetKey(KeyCode.UpArrow))
			transform.Rotate(Vector3.right *moveSpeed * Time.deltaTime);
		
		if (Input.GetKey(KeyCode.DownArrow))
			transform.Rotate(-Vector3.right *moveSpeed * Time.deltaTime);
				
		if (Input.GetKey(KeyCode.RightArrow))
			transform.Rotate(Vector3.up *turnSpeed * Time.deltaTime);
		
		if (Input.GetKey(KeyCode.LeftArrow))
			transform.Rotate(Vector3.up *-turnSpeed * Time.deltaTime);		
    }
	
	void Movimiento (int distancia)	{
		if(distancia <= 10)
		{				
			if (Input.GetKey(KeyCode.Space))
			transform.Rotate(Vector3.forward *turnSpeed * Time.deltaTime);
		}
		
		
		if(distancia >10 & distancia<=20)
		{				
			if (Input.GetKey(KeyCode.Escape))
			transform.Rotate(-Vector3.forward *turnSpeed * Time.deltaTime);
		}
		
		if(distancia >20 & distancia<=30)
		{				
			if (Input.GetKey(KeyCode.UpArrow))
			transform.Rotate(Vector3.right *moveSpeed * Time.deltaTime);
		}
		
		if(distancia >30 & distancia<=40)
		{				
			if (Input.GetKey(KeyCode.DownArrow))
			transform.Rotate(-Vector3.right *moveSpeed * Time.deltaTime);
		}
		
		
		if(distancia >40 & distancia<=50)
		{				
			if (Input.GetKey(KeyCode.RightArrow))
			transform.Rotate(Vector3.up *turnSpeed * Time.deltaTime);
		}
		
		if(distancia >50 & distancia<=60)
		{				
				if (Input.GetKey(KeyCode.LeftArrow))
				transform.Rotate(Vector3.up *-turnSpeed * Time.deltaTime);	
		}
	}
	
}