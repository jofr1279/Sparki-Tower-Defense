using UnityEngine;
using UnityEngine.UI;

public class UIController : MonoBehaviour {
    public InputField addressInput;
    public Controller controller;

    public void Connect() {
        controller.SendMessage("Connect", addressInput.text);
    }
}